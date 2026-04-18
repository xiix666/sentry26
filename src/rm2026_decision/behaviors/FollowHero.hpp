#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>

namespace rm2026_decision {

/**
 * @brief 跟踪我方英雄的行为节点
 * 以英雄位置为圆心，生成候选点进行跟踪
 */
class FollowHero : public BT::StatefulActionNode
{
public:
    FollowHero(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , node_(nullptr)
        , navigation_started_(false)
        , target_selected_(false)
        , rng_(std::random_device{}())
        , center_updated_(false)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2节点实例"),
            BT::InputPort<float>("current_pose_x", "@current_pose_x", "当前机器人X坐标(米)"),
            BT::InputPort<float>("current_pose_y", "@current_pose_y", "当前机器人Y坐标(米)"),
            BT::InputPort<float>("hero_x", "@teammate_hero_location_x", "英雄X坐标"),
            BT::InputPort<float>("hero_y", "@teammate_hero_location_y", "英雄Y坐标"),
            BT::InputPort<float>("center_x", "@hero_follow_center_x", "跟踪圆心X坐标"),
            BT::InputPort<float>("center_y", "@hero_follow_center_y", "跟踪圆心Y坐标"),
            BT::InputPort<double>("tolerance", 0.1, "到达目标点容忍距离(米)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_goal", "@nav_goal", "导航目标(用于距离判断, inout)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goalOut", "导航目标"),
            BT::OutputPort<bool>("nav_goal_requested", "导航目标请求"),
            BT::OutputPort<float>("center_x_out", "更新跟踪圆心X坐标"),
            BT::OutputPort<float>("center_y_out", "更新跟踪圆心Y坐标"),
        };
    }

    BT::NodeStatus onStart() override
    {
        // 获取ROS2节点
        auto node_input = getInput<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_input) {
            RCLCPP_ERROR(rclcpp::get_logger("FollowHero"), "Failed to get ROS2 node instance");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        // 获取当前位置
        auto current_x = getInput<float>("current_pose_x");
        auto current_y = getInput<float>("current_pose_y");
        if (!current_x || !current_y) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot position");
            return BT::NodeStatus::FAILURE;
        }
        robot_x_ = current_x.value();
        robot_y_ = current_y.value();

        // 获取英雄位置
        auto hero_x = getInput<float>("hero_x");
        auto hero_y = getInput<float>("hero_y");
        if (!hero_x || !hero_y) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get hero position");
            return BT::NodeStatus::FAILURE;
        }
        float hero_x_val = hero_x.value();
        float hero_y_val = hero_y.value();

        // 获取上次的圆心位置
        auto center_x = getInput<float>("center_x");
        auto center_y = getInput<float>("center_y");
        if (center_x && center_y) {
            float last_center_x = center_x.value();
            float last_center_y = center_y.value();

            // 计算英雄移动距离
            float distance = std::sqrt(std::pow(hero_x_val - last_center_x, 2) +
                                     std::pow(hero_y_val - last_center_y, 2));

            // 如果英雄移动距离小于0.3m，认为位置变动较小，直接返回SUCCESS
            if (distance < 0.3f) {
                RCLCPP_INFO(node_->get_logger(), "英雄位置变动较小(%.2fm)，跳过跟踪", distance);
                return BT::NodeStatus::SUCCESS;
            }
        }

        // 更新圆心位置
        center_x_ = hero_x_val;
        center_y_ = hero_y_val;
        center_updated_ = true;

        // 获取容忍距离
        auto tolerance = getInput<double>("tolerance");
        tolerance_ = tolerance.value_or(0.1);

        // 生成候选点
        if (!generateCandidatePoints()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate candidate points");
            return BT::NodeStatus::FAILURE;
        }

        // 筛选可行点
        filterFeasiblePoints();

        if (feasible_points_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No feasible points, skipping tracking");
            return BT::NodeStatus::SUCCESS;
        }

        // 随机选择目标点
        selectRandomTarget();
        navigateToTarget();

        RCLCPP_INFO(node_->get_logger(), "Start tracking hero, target point: (%.2f, %.2f)",
                   target_pose_.pose.position.x, target_pose_.pose.position.y);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

        // 获取英雄最新位置
        auto hero_x = getInput<float>("hero_x");
        auto hero_y = getInput<float>("hero_y");
        if (hero_x && hero_y) {
            float current_hero_x = hero_x.value();
            float current_hero_y = hero_y.value();

            // 计算英雄与圆心的距离
            float distance = std::sqrt(std::pow(current_hero_x - center_x_, 2) +
                                     std::pow(current_hero_y - center_y_, 2));

            // 如果英雄移动距离大于1m，中断跟踪
            if (distance > 1.0f) {
                RCLCPP_INFO(node_->get_logger(), "英雄移动距离过大(%.2fm)，中断跟踪", distance);
                return BT::NodeStatus::SUCCESS;
            }
        }

        // 检查是否到达目标点（从黑板变量 nav_goal 读取目标位姿, inout）
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
                if (center_updated_) {
                    setOutput("center_x_out", center_x_);
                    setOutput("center_y_out", center_y_);
                }
                RCLCPP_INFO(node_->get_logger(), "Reached tracking target point");
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        navigation_started_ = false;
        target_selected_ = false;

        // 如果圆心已更新，输出到黑板
        if (center_updated_) {
            setOutput("center_x_out", center_x_);
            setOutput("center_y_out", center_y_);
        }
    }

private:
    bool generateCandidatePoints()
    {
        candidate_points_.clear();

        // 计算机器人到英雄的方向
        float dx_to_hero = center_x_ - robot_x_;
        float dy_to_hero = center_y_ - robot_y_;
        float distance_to_hero = std::sqrt(dx_to_hero * dx_to_hero + dy_to_hero * dy_to_hero);

        if (distance_to_hero < 0.1f) {
            RCLCPP_WARN(node_->get_logger(), "机器人与英雄距离过近，无法生成候选点");
            return false;
        }

        // 计算到机器人的角度
        float angle_to_robot = std::atan2(robot_y_ - center_y_, robot_x_ - center_x_);

        // 确定最近的半圆弧范围 (90度范围)
        float start_angle = angle_to_robot - M_PI/4;  // -45度
        float end_angle = angle_to_robot + M_PI/4;     // +45度

        // 生成9个候选点
        float angle_step = (end_angle - start_angle) / 8;  // 9个点需要8个间隔

        for (int i = 0; i < 9; ++i) {
            float angle = start_angle + i * angle_step;

            geometry_msgs::msg::Point point;
            point.x = center_x_ + 0.5f * std::cos(angle);
            point.y = center_y_ + 0.5f * std::sin(angle);
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

        // 地图外为禁区（与 decision_node 中场地范围一致：x in [0,12], y in [0,8]）
        const float map_x_min = 0.0f, map_x_max = 12.0f;
        const float map_y_min = 0.0f, map_y_max = 8.0f;
        if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max) {
            return true;
        }
        // 禁区1: (2.3,8.0), (2.3,2.8), (4.7,2.8), (4.7,8.0)
        std::vector<std::pair<float, float>> zone1 = {
            {2.3f, 8.0f}, {2.3f, 2.8f}, {4.7f, 2.8f}, {4.7f, 8.0f}
        };

        // 禁区2: (7.3,5.2), (7.3,0.0), (9.7,0.0), (9.7,5.2)
        std::vector<std::pair<float, float>> zone2 = {
            {7.3f, 5.2f}, {7.3f, 0.0f}, {9.7f, 0.0f}, {9.7f, 5.2f}
        };

        return isPointInPolygon(x, y, zone1) || isPointInPolygon(x, y, zone2);
    }

    bool isPointInPolygon(float x, float y, const std::vector<std::pair<float, float>>& polygon)
    {
        // 复用decision_node.cpp中的射线法实现
        bool inside = false;
        size_t n = polygon.size();

        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float x1 = polygon[i].first, y1 = polygon[i].second;
            float x2 = polygon[j].first, y2 = polygon[j].second;

            if (((y1 > y) != (y2 > y))) {
                if (y1 != y2) {
                    float intersect_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
                    if (x < intersect_x) {
                        inside = !inside;
                    }
                }
            }
        }

        return inside;
    }

    void selectRandomTarget()
    {
        if (feasible_points_.empty()) {
            return;
        }

        std::uniform_int_distribution<size_t> dist(0, feasible_points_.size() - 1);
        size_t random_index = dist(rng_);

        target_pose_.header.frame_id = "map";
        target_pose_.header.stamp = node_->get_clock()->now();
        target_pose_.pose.position = feasible_points_[random_index];
        target_pose_.pose.orientation.w = 1.0;
        target_pose_.pose.orientation.z = 0.0;

        target_selected_ = true;
    }

    void navigateToTarget()
    {
        if (!target_selected_) {
            return;
        }

        setOutput("nav_goalOut", target_pose_);
        setOutput("nav_goal_requested", true);
        navigation_started_ = true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool navigation_started_;
    bool target_selected_;
    std::mt19937 rng_;
    bool center_updated_;

    float robot_x_, robot_y_;
    float center_x_, center_y_;
    float tolerance_;

    geometry_msgs::msg::PoseStamped target_pose_;
    std::vector<geometry_msgs::msg::Point> candidate_points_;
    std::vector<geometry_msgs::msg::Point> feasible_points_;
};

}  // namespace rm2026_decision