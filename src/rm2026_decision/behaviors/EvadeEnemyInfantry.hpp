#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

namespace rm2026_decision {

/**
 * @brief 防止敌方步兵近身的异步行为节点
 * 当视角内检测到敌方步兵且距离<1m时，在背对敌人的260°弧上选点远离，目标点为可行点中离“小1级区域期望点”最近的点
 */
class EvadeEnemyInfantry : public BT::StatefulActionNode
{
public:
    EvadeEnemyInfantry(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , node_(nullptr)
        , navigation_started_(false)
        , target_selected_(false)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2节点实例"),
            BT::InputPort<float>("current_pose_x", "@current_pose_x", "当前机器人X坐标(米)"),
            BT::InputPort<float>("current_pose_y", "@current_pose_y", "当前机器人Y坐标(米)"),
            BT::InputPort<float>("enemy_infantry1_distance", "@enemy_infantry1_distance", "敌方步兵1距离(-1未检测)"),
            BT::InputPort<float>("enemy_infantry2_distance", "@enemy_infantry2_distance", "敌方步兵2距离(-1未检测)"),
            BT::InputPort<float>("enemy_infantry1_x", "@enemy_infantry1_x", "敌方步兵1 X"),
            BT::InputPort<float>("enemy_infantry1_y", "@enemy_infantry1_y", "敌方步兵1 Y"),
            BT::InputPort<float>("enemy_infantry2_x", "@enemy_infantry2_x", "敌方步兵2 X"),
            BT::InputPort<float>("enemy_infantry2_y", "@enemy_infantry2_y", "敌方步兵2 Y"),
            BT::InputPort<int>("current_area", "@current_area", "当前区域ID"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("supply_point_location", "@supply_point_location", "补给点(区域0/1期望点)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goto_area1", "@goto_area1", "区域2期望点"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goto_area2", "@goto_area2", "区域3期望点"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goto_area3", "@goto_area3", "区域4期望点"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("center_capture_point", "@center_capture_point", "区域5/6/7期望点"),
            BT::InputPort<double>("tolerance", 0.1, "到达目标点容忍距离(米)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_goal", "@nav_goal", "导航目标(用于距离判断, inout)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goalOut", "导航目标"),
            BT::OutputPort<bool>("nav_goal_requested", "导航目标请求"),
        };
    }

    BT::NodeStatus onStart() override
    {
        auto node_input = getInput<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_input) {
            RCLCPP_ERROR(rclcpp::get_logger("EvadeEnemyInfantry"), "无法获取ROS2节点实例");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        auto current_x = getInput<float>("current_pose_x");
        auto current_y = getInput<float>("current_pose_y");
        if (!current_x || !current_y) {
            RCLCPP_ERROR(node_->get_logger(), "无法获取当前机器人位置");
            return BT::NodeStatus::FAILURE;
        }
        robot_x_ = current_x.value();
        robot_y_ = current_y.value();

        auto d1 = getInput<float>("enemy_infantry1_distance");
        auto d2 = getInput<float>("enemy_infantry2_distance");
        float dist1 = d1 ? d1.value() : -1.0f;
        float dist2 = d2 ? d2.value() : -1.0f;

        // 触发条件：至少一名步兵被检测到且最近距离 < 1m
        bool detected1 = (dist1 >= 0.0f && dist1 < 1.0f);
        bool detected2 = (dist2 >= 0.0f && dist2 < 1.0f);
        if (!detected1 && !detected2) {
            RCLCPP_DEBUG(node_->get_logger(), "无敌方步兵近身(1m内)，不执行远离");
            return BT::NodeStatus::FAILURE;
        }

        // 选最近的步兵作为“要背对的敌人”
        float enemy_x, enemy_y;
        if (detected1 && detected2) {
            if (dist1 <= dist2) {
                auto x1 = getInput<float>("enemy_infantry1_x");
                auto y1 = getInput<float>("enemy_infantry1_y");
                enemy_x = x1 ? x1.value() : robot_x_;
                enemy_y = y1 ? y1.value() : robot_y_;
            } else {
                auto x2 = getInput<float>("enemy_infantry2_x");
                auto y2 = getInput<float>("enemy_infantry2_y");
                enemy_x = x2 ? x2.value() : robot_x_;
                enemy_y = y2 ? y2.value() : robot_y_;
            }
        } else if (detected1) {
            auto x1 = getInput<float>("enemy_infantry1_x");
            auto y1 = getInput<float>("enemy_infantry1_y");
            enemy_x = x1 ? x1.value() : robot_x_;
            enemy_y = y1 ? y1.value() : robot_y_;
        } else {
            auto x2 = getInput<float>("enemy_infantry2_x");
            auto y2 = getInput<float>("enemy_infantry2_y");
            enemy_x = x2 ? x2.value() : robot_x_;
            enemy_y = y2 ? y2.value() : robot_y_;
        }

        auto tolerance = getInput<double>("tolerance");
        tolerance_ = tolerance.value_or(0.1);

        if (!generateCandidatePoints(enemy_x, enemy_y)) {
            RCLCPP_ERROR(node_->get_logger(), "无法生成远离候选点");
            return BT::NodeStatus::FAILURE;
        }

        filterFeasiblePoints();
        if (feasible_points_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "没有可行远离点");
            return BT::NodeStatus::FAILURE;
        }

        if (!selectBestTarget()) {
            RCLCPP_WARN(node_->get_logger(), "无法选取最优点");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("nav_goalOut", target_pose_);
        setOutput("nav_goal_requested", true);
        navigation_started_ = true;
        target_selected_ = true;

        RCLCPP_INFO(node_->get_logger(), "EvadeEnemyInfantry: 远离敌方步兵，目标点 (%.2f, %.2f)",
                    target_pose_.pose.position.x, target_pose_.pose.position.y);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

        // 可选：若敌人已离开1m则提前结束
        auto d1 = getInput<float>("enemy_infantry1_distance");
        auto d2 = getInput<float>("enemy_infantry2_distance");
        float dist1 = d1 ? d1.value() : -1.0f;
        float dist2 = d2 ? d2.value() : -1.0f;
        bool still_near1 = (dist1 >= 0.0f && dist1 < 1.0f);
        bool still_near2 = (dist2 >= 0.0f && dist2 < 1.0f);
        if (!still_near1 && !still_near2) {
            RCLCPP_INFO(node_->get_logger(), "敌方步兵已离开1m，结束远离");
            return BT::NodeStatus::SUCCESS;
        }

        auto current_x = getInput<float>("current_pose_x");
        auto current_y = getInput<float>("current_pose_y");
        auto nav_goal_input = getInput<geometry_msgs::msg::PoseStamped>("nav_goal");
        if (current_x && current_y && nav_goal_input) {
            float target_x = nav_goal_input.value().pose.position.x;
            float target_y = nav_goal_input.value().pose.position.y;
            float dx = current_x.value() - target_x;
            float dy = current_y.value() - target_y;
            float distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= tolerance_) {
                RCLCPP_INFO(node_->get_logger(), "已到达远离目标点");
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        navigation_started_ = false;
        target_selected_ = false;
    }

private:
    static constexpr float DANGER_RADIUS = 1.0f;   // 近身判定半径 1m
    static constexpr float EVADE_RADIUS = 2.0f;    // 远离圆半径 2m
    static constexpr int NUM_CANDIDATES = 13;       // 260°弧分13段

    bool generateCandidatePoints(float enemy_x, float enemy_y)
    {
        candidate_points_.clear();

        float dx = enemy_x - robot_x_;
        float dy = enemy_y - robot_y_;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 1e-6f) {
            RCLCPP_WARN(node_->get_logger(), "敌我距离过近，无法确定背对方向");
            return false;
        }

        // 指向敌人的方向
        float angle_to_enemy = std::atan2(dy, dx);
        // 背对弧：中心 angle_to_enemy + π，跨度 260°（左右各 130°）
        float start_angle = angle_to_enemy + static_cast<float>(M_PI) - 130.0f * static_cast<float>(M_PI) / 180.0f;
        float angle_step = (260.0f * static_cast<float>(M_PI) / 180.0f) / static_cast<float>(NUM_CANDIDATES);

        for (int i = 0; i < NUM_CANDIDATES; ++i) {
            float angle = start_angle + (static_cast<float>(i) + 0.5f) * angle_step;
            geometry_msgs::msg::Point point;
            point.x = robot_x_ + EVADE_RADIUS * std::cos(angle);
            point.y = robot_y_ + EVADE_RADIUS * std::sin(angle);
            point.z = 0.0f;
            candidate_points_.push_back(point);
        }
        return true;
    }

    void filterFeasiblePoints()
    {
        feasible_points_.clear();
        for (const auto& point : candidate_points_) {
            if (!isPointInForbiddenZone(point.x, point.y)) {
                feasible_points_.push_back(point);
            }
        }
    }

    bool isPointInForbiddenZone(float x, float y)
    {
        const float map_x_min = 0.0f, map_x_max = 12.0f;
        const float map_y_min = 0.0f, map_y_max = 8.0f;
        if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max) {
            return true;
        }
        std::vector<std::pair<float, float>> zone1 = {
            {2.3f, 8.0f}, {2.3f, 2.8f}, {4.7f, 2.8f}, {4.7f, 8.0f}
        };
        std::vector<std::pair<float, float>> zone2 = {
            {7.3f, 5.2f}, {7.3f, 0.0f}, {9.7f, 0.0f}, {9.7f, 5.2f}
        };
        return isPointInPolygon(x, y, zone1) || isPointInPolygon(x, y, zone2);
    }

    static bool isPointInPolygon(float x, float y, const std::vector<std::pair<float, float>>& polygon)
    {
        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float x1 = polygon[i].first, y1 = polygon[i].second;
            float x2 = polygon[j].first, y2 = polygon[j].second;
            if (((y1 > y) != (y2 > y)) && y1 != y2) {
                float intersect_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
                if (x < intersect_x) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    bool selectBestTarget()
    {
        float expected_x, expected_y;
        if (!getExpectedPoint(expected_x, expected_y)) {
            return false;
        }

        size_t best_idx = 0;
        float best_dist_sq = std::numeric_limits<float>::max();
        for (size_t i = 0; i < feasible_points_.size(); ++i) {
            float dx = feasible_points_[i].x - expected_x;
            float dy = feasible_points_[i].y - expected_y;
            float d2 = dx * dx + dy * dy;
            if (d2 < best_dist_sq) {
                best_dist_sq = d2;
                best_idx = i;
            }
        }

        target_pose_.header.frame_id = "map";
        target_pose_.header.stamp = node_->get_clock()->now();
        target_pose_.pose.position = feasible_points_[best_idx];
        target_pose_.pose.orientation.w = 1.0;
        target_pose_.pose.orientation.z = 0.0;
        return true;
    }

    bool getExpectedPoint(float& out_x, float& out_y)
    {
        auto area_input = getInput<int>("current_area");
        int area = area_input ? area_input.value() : -1;

        if (area == 0 || area == 1) {
            auto pose = getInput<geometry_msgs::msg::PoseStamped>("supply_point_location");
            if (!pose) return false;
            out_x = static_cast<float>(pose.value().pose.position.x);
            out_y = static_cast<float>(pose.value().pose.position.y);
            return true;
        }
        if (area == 2) {
            auto pose = getInput<geometry_msgs::msg::PoseStamped>("goto_area1");
            if (!pose) return false;
            out_x = static_cast<float>(pose.value().pose.position.x);
            out_y = static_cast<float>(pose.value().pose.position.y);
            return true;
        }
        if (area == 3) {
            auto pose = getInput<geometry_msgs::msg::PoseStamped>("goto_area2");
            if (!pose) return false;
            out_x = static_cast<float>(pose.value().pose.position.x);
            out_y = static_cast<float>(pose.value().pose.position.y);
            return true;
        }
        // if (area == 4) {
        //     auto pose = getInput<geometry_msgs::msg::PoseStamped>("goto_area3");
        //     if (!pose) return false;
        //     out_x = static_cast<float>(pose.value().pose.position.x);
        //     out_y = static_cast<float>(pose.value().pose.position.y);
        //     return true;
        // }
        if (area == 5 || area == 6 || area == 7 || area == 4) {
            auto pose = getInput<geometry_msgs::msg::PoseStamped>("center_capture_point");
            if (!pose) return false;
            out_x = static_cast<float>(pose.value().pose.position.x);
            out_y = static_cast<float>(pose.value().pose.position.y);
            return true;
        }
        // 区域 -1 或其他：默认用补给点
        auto pose = getInput<geometry_msgs::msg::PoseStamped>("supply_point_location");
        if (!pose) return false;
        out_x = static_cast<float>(pose.value().pose.position.x);
        out_y = static_cast<float>(pose.value().pose.position.y);
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool navigation_started_;
    bool target_selected_;
    float robot_x_, robot_y_;
    float tolerance_;
    geometry_msgs::msg::PoseStamped target_pose_;
    std::vector<geometry_msgs::msg::Point> candidate_points_;
    std::vector<geometry_msgs::msg::Point> feasible_points_;
};

}  // namespace rm2026_decision
