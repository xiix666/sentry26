#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>
#include <cmath>

namespace rm2026_decision {

/**
 * @brief 待在补给点的异步行为节点
 * 导航到补给区中心并持续监控导航状态，确保哨兵停留在补给点
 */
class StayAtSupplyPoint : public BT::StatefulActionNode
{
public:
    StayAtSupplyPoint(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , node_(nullptr)
        , navigation_started_(false)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("supply_point_location", "Supply point location"),
            BT::InputPort<double>("timeout", 15.0, "Navigation timeout(seconds)"),
            BT::InputPort<double>("tolerance", 0.2, "Tolerance distance to target point(meters)"),
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
            RCLCPP_ERROR(rclcpp::get_logger("StayAtSupplyPoint"), "Failed to get ROS2 node instance");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        // 获取补给点位置
        auto supply_point = getInput<geometry_msgs::msg::PoseStamped>("supply_point_location");
        if (!supply_point) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get supply point location: supply_point_location");
            return BT::NodeStatus::FAILURE;
        }

        // 设置超时时间
        auto timeout = getInput<double>("timeout");
        timeout_point_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timeout.value_or(15.0)));

        // 获取容忍距离
        auto tolerance = getInput<double>("tolerance");
        tolerance_ = tolerance.value_or(0.2);

        // 通过输出端口设置导航目标（写入黑板 nav_goal，距离判断从 nav_goal 端口读取, input/output分离）
        setOutput("nav_goalOut", supply_point.value());
        setOutput("nav_goal_requested", true);

        navigation_started_ = true;

        RCLCPP_INFO(node_->get_logger(), "开始导航到补给区中心 (x=%.2f, y=%.2f)",
                   supply_point.value().pose.position.x,
                   supply_point.value().pose.position.y);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

        // 检查是否超时
        if (std::chrono::steady_clock::now() > timeout_point_) {
            RCLCPP_WARN(node_->get_logger(), "Navigation to supply point timed out");
            return BT::NodeStatus::FAILURE;
        }

        // 仅按当前点到目标点距离判断到达（不依赖 navigation/status）
        if (checkDistanceToTarget()) {
            RCLCPP_INFO(node_->get_logger(), "Reached supply point");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "Navigation to supply point interrupted");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("StayAtSupplyPoint"), "Navigation to supply point interrupted (node not initialized)");
        }
        navigation_started_ = false;
    }

private:
    bool checkDistanceToTarget()
    {
        if (!node_) {
            return false;
        }
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

        float dx = target_x - current_x;
        float dy = target_y - current_y;
        float distance = std::sqrt(dx * dx + dy * dy);
        return distance <= tolerance_;
    }

    rclcpp::Node::SharedPtr node_;
    std::chrono::steady_clock::time_point timeout_point_;
    double tolerance_;
    bool navigation_started_;
};

} // namespace rm2026_decision