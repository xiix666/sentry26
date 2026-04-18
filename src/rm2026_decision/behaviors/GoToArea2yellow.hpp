#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>
#include <cmath>

namespace rm2026_decision {

/**
 * @brief 前往区域2黄色点的异步行为节点
 * 导航到区域2黄色点并持续监控导航状态，距离目标点<=0.3m视为到达
 */
class GoToArea2yellow : public BT::StatefulActionNode
{
public:
    GoToArea2yellow(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , node_(nullptr)
        , navigation_started_(false)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Area2yellow position"),
            BT::InputPort<double>("timeout", 15.0, "Navigation timeout(seconds)"),
            BT::InputPort<double>("tolerance", 0.3, "Tolerance distance to target point(meters)"),
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2 node instance"),
            BT::InputPort<float>("current_pose_x", "@current_pose_x", "Current robot X coordinate(meters)"),
            BT::InputPort<float>("current_pose_y", "@current_pose_y", "Current robot Y coordinate(meters)"),
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
            RCLCPP_ERROR(rclcpp::get_logger("GoToArea2yellow"), "Failed to get ROS2 node instance");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        // 获取目标位置
        auto target_pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
        if (!target_pose) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get Area2yellow target position: target_pose");
            return BT::NodeStatus::FAILURE;
        }

        // 设置超时时间
        auto timeout = getInput<double>("timeout");
        timeout_point_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timeout.value_or(15.0)));

        // 获取容忍距离
        auto tolerance = getInput<double>("tolerance");
        tolerance_ = tolerance.value_or(0.3);

        // 通过输出端口设置导航目标（写入黑板 nav_goal，距离判断从 nav_goal 端口读取, input/output分离）
        setOutput("nav_goalOut", target_pose.value());
        setOutput("nav_goal_requested", true);

        navigation_started_ = true;

        RCLCPP_INFO(node_->get_logger(), "Start navigating to area2yellow (x=%.2f, y=%.2f), tolerance=%.2fm",
                   target_pose.value().pose.position.x,
                   target_pose.value().pose.position.y,
                   tolerance_);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

        // 检查是否超时
        if (std::chrono::steady_clock::now() > timeout_point_) {
            RCLCPP_WARN(node_->get_logger(), "Navigation to Area2yellow timed out");
            return BT::NodeStatus::FAILURE;
        }

        // 仅按当前点到目标点距离判断到达（不依赖 navigation/status）
        if (checkDistanceToTarget()) {
            RCLCPP_INFO(node_->get_logger(), "Reached Area2yellow");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "Navigation to Area2yellow interrupted");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("GoToArea2yellow"), "Navigation to Area2yellow interrupted (node not initialized)");
        }
        navigation_started_ = false;
    }

private:
    bool checkDistanceToTarget()
    {
        if (!node_) {
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

        RCLCPP_DEBUG(node_->get_logger(), "Current position(%.2f, %.2f), target position(%.2f, %.2f), distance=%.2fm",
                    current_x, current_y, target_x, target_y, distance);

        return distance <= tolerance_;
    }

    rclcpp::Node::SharedPtr node_;
    std::chrono::steady_clock::time_point timeout_point_;
    double tolerance_;
    bool navigation_started_;
};

} // namespace rm2026_decision