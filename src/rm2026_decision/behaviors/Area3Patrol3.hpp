#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>
#include <random>
#include <cmath>
#include <thread>

namespace rm2026_decision {

/**
 * @brief 区域3三点随机巡航的异步行为节点
 * 在三个点之间随机巡航，到达一点后重新随机选择下一目标点
 */
class Area3Patrol3 : public BT::StatefulActionNode
{
public:
    Area3Patrol3(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , node_(nullptr)
        , navigation_started_(false)
        , current_target_index_(-1)
        , rng_(std::random_device{}())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("point1", "Area3 first patrol point"),   
            BT::InputPort<geometry_msgs::msg::PoseStamped>("point2", "Area3 second patrol point"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("point3", "Area3 third patrol point"),
            BT::InputPort<double>("timeout", 15.0, "Navigation timeout(seconds)"),
            BT::InputPort<double>("tolerance", 0.1, "Tolerance distance to target point(meters)"),
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2 node instance"),
            BT::InputPort<float>("current_pose_x", "@current_pose_x", "Current robot X coordinate(meters)"),
            BT::InputPort<float>("current_pose_y", "@current_pose_y", "Current robot Y coordinate(meters)"),
            BT::InputPort<int>("center_status", "@center_status", "Center status"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_goal", "@nav_goal", "Navigation goal(used for distance judgment, inout)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goalOut", "Navigation goal"),
            BT::OutputPort<bool>("nav_goal_requested", "Navigation goal requested"),
        };
    }

    BT::NodeStatus onStart() override
    {
        // 获取ROS2节点
        auto node_input = getInput<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_input) {
            RCLCPP_ERROR(rclcpp::get_logger("Area3Patrol3"), "Failed to get ROS2 node instance");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        // 获取三个巡逻点
        auto point1 = getInput<geometry_msgs::msg::PoseStamped>("point1");
        auto point2 = getInput<geometry_msgs::msg::PoseStamped>("point2");
        auto point3 = getInput<geometry_msgs::msg::PoseStamped>("point3");

        if (!point1 || !point2 || !point3) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get Area3 patrol points");
            return BT::NodeStatus::FAILURE;
        }

        patrol_points_[0] = point1.value();
        patrol_points_[1] = point2.value();
        patrol_points_[2] = point3.value();

        // 获取容忍距离
        auto tolerance = getInput<double>("tolerance");
        tolerance_ = tolerance.value_or(0.2);

        // 随机选择第一个目标点
        selectRandomTarget();
        navigateToCurrentTarget();
        waiting_for_next_target_ = false;

        RCLCPP_INFO(node_->get_logger(), "Start Area3 three-point random patrol, first target point: %d", current_target_index_);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

        // 调试信息：确认Area3Patrol3节点正在运行

        // 检查是否超时
        if (std::chrono::steady_clock::now() > timeout_point_) {
            RCLCPP_WARN(node_->get_logger(), "Navigation to patrol point timed out, reselect target point");
            selectRandomTarget();
            navigateToCurrentTarget();
            return BT::NodeStatus::RUNNING;
        }

        // 仅按当前点到目标点距离判断到达（不依赖 navigation/status）
        if (waiting_for_next_target_) {
            // 到达后等待 5s 再切换新点，避免在到达期间反复触发随机选择
            auto now = std::chrono::steady_clock::now();
            if (now >= reached_target_time_ + std::chrono::seconds(5)) {
                waiting_for_next_target_ = false;
                RCLCPP_INFO(node_->get_logger(), "Waited 5s after reaching patrol point, select next random target");
                selectRandomTarget();
                navigateToCurrentTarget();
            }
            return BT::NodeStatus::RUNNING;
        }

        if (checkDistanceToTarget()) {
            RCLCPP_INFO(node_->get_logger(), "Reached patrol point %d, start 5s wait before selecting next", current_target_index_);
            waiting_for_next_target_ = true;
            reached_target_time_ = std::chrono::steady_clock::now();
            return BT::NodeStatus::RUNNING;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "Area3 patrol interrupted");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("Area3Patrol3"), "Area3 patrol interrupted (node not initialized)");
        }
        navigation_started_ = false;
        current_target_index_ = -1;
        waiting_for_next_target_ = false;
    }

private:
    void selectRandomTarget()
    {
        std::uniform_int_distribution<int> dist(0, 2);
        int new_target;

        // 避免连续选择同一个点
        do {
            new_target = dist(rng_);
        } while (new_target == current_target_index_ && patrol_points_count_ > 1);

        current_target_index_ = new_target;

        if (node_) {
            RCLCPP_DEBUG(node_->get_logger(), "Randomly select patrol point: %d", current_target_index_);
        }
    }

    void navigateToCurrentTarget()
    {
        if (!node_) {
            return;
        }

        if (current_target_index_ < 0 || current_target_index_ >= 3) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid patrol point index: %d", current_target_index_);
            return;
        }

        // 设置超时时间
        auto timeout = getInput<double>("timeout");
        timeout_point_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timeout.value_or(15.0)));

        // 通过黑板请求导航目标发布
        auto target_pose = patrol_points_[current_target_index_];
        setOutput("nav_goalOut", target_pose);
        setOutput("nav_goal_requested", true);

        navigation_started_ = true;

        RCLCPP_INFO(node_->get_logger(), "Navigate to patrol point %d (x=%.2f, y=%.2f), tolerance=%.2fm",
                   current_target_index_,
                   patrol_points_[current_target_index_].pose.position.x,
                   patrol_points_[current_target_index_].pose.position.y,
                   tolerance_);
    }

    bool checkDistanceToTarget()
    {
        if (!node_) {
            return false;
        }

        if (current_target_index_ < 0 || current_target_index_ >= 3) {
            return false;
        }

        // 获取当前机器人位置
        auto current_x_input = getInput<float>("current_pose_x");
        auto current_y_input = getInput<float>("current_pose_y");

        if (!current_x_input || !current_y_input) {
            RCLCPP_WARN(node_->get_logger(), "Failed to get current robot position");
            return false;
        }

        float current_x = current_x_input.value();
        float current_y = current_y_input.value();

        // 从黑板变量 nav_goal 读取目标位姿 (inout)
        auto nav_goal_input = getInput<geometry_msgs::msg::PoseStamped>("nav_goal");
        if (!nav_goal_input) {
            RCLCPP_WARN(node_->get_logger(), "Failed to get nav_goal from blackboard");
            return false;
        }
        float target_x = nav_goal_input.value().pose.position.x;
        float target_y = nav_goal_input.value().pose.position.y;

        // 计算距离
        float dx = target_x - current_x;
        float dy = target_y - current_y;
        float distance = std::sqrt(dx * dx + dy * dy);

        RCLCPP_DEBUG(node_->get_logger(), "Distance check: current position(%.2f, %.2f), target position(%.2f, %.2f), distance=%.2fm, tolerance=%.2fm, result=%s",
                    current_x, current_y, target_x, target_y, distance, tolerance_, (distance <= tolerance_) ? "通过" : "不通过");

        return distance <= tolerance_;
    }

    rclcpp::Node::SharedPtr node_;
    std::array<geometry_msgs::msg::PoseStamped, 3> patrol_points_;
    std::chrono::steady_clock::time_point timeout_point_;
    double tolerance_;
    bool navigation_started_;
    int current_target_index_;
    std::mt19937 rng_;
    static constexpr int patrol_points_count_ = 3;

    // 到达当前选点后，等待 5 秒再随机选新点
    bool waiting_for_next_target_{false};
    std::chrono::steady_clock::time_point reached_target_time_{};
};

} // namespace rm2026_decision