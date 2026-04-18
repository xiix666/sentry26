#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <atomic>
#include <chrono>
#include <cmath>

namespace rm2026_decision {

/**
 * @brief 区域2黄色点停留30秒计时器条件节点
 * 当哨兵在区域2黄色点0.2m范围内停留超过30秒时返回SUCCESS
 */
class Area2YellowTimer : public BT::ConditionNode
{
public:
    Area2YellowTimer(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config)
        , node_(nullptr)
        , stay_duration_(10.0)  // 30秒
        , tolerance_(0.2)       // 0.2m容忍度
        , is_in_zone_(false)
        , timer_start_(std::chrono::steady_clock::time_point::min())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("yellow_point", "区域2黄色点位置"),
            BT::InputPort<float>("robot_x", "@current_pose_x", "机器人X坐标"),
            BT::InputPort<float>("robot_y", "@current_pose_y", "机器人Y坐标"),
            BT::InputPort<double>("stay_duration", 10.0, "停留时间(秒)"),
            BT::InputPort<double>("tolerance", 0.2, "位置容忍距离(米)"),
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2节点实例"),
        };
    }

    BT::NodeStatus tick() override
    {
        // 获取ROS2节点
        auto node_input = getInput<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_input || !node_input.value()) {
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        // 获取当前位置
        auto robot_x_input = getInput<float>("robot_x");
        auto robot_y_input = getInput<float>("robot_y");
        if (!robot_x_input || !robot_y_input) {
            if (node_) {
                RCLCPP_WARN(node_->get_logger(), "无法获取机器人当前位置");
            }
            return BT::NodeStatus::FAILURE;
        }

        float robot_x = robot_x_input.value();
        float robot_y = robot_y_input.value();

        // 获取黄色点位置
        auto yellow_point = getInput<geometry_msgs::msg::PoseStamped>("yellow_point");
        if (!yellow_point) {
            if (node_) {
                RCLCPP_ERROR(node_->get_logger(), "无法获取区域2黄色点位置");
            }
            return BT::NodeStatus::FAILURE;
        }

        float target_x = yellow_point.value().pose.position.x;
        float target_y = yellow_point.value().pose.position.y;

        // 获取参数
        auto stay_duration_input = getInput<double>("stay_duration");
        auto tolerance_input = getInput<double>("tolerance");

        if (stay_duration_input) stay_duration_ = stay_duration_input.value();
        if (tolerance_input) tolerance_ = tolerance_input.value();

        // 计算距离
        float dx = target_x - robot_x;
        float dy = target_y - robot_y;
        float distance = std::sqrt(dx * dx + dy * dy);

        bool currently_in_zone = (distance <= tolerance_);

        // 状态变化处理
        if (currently_in_zone && !is_in_zone_) {
            // 刚进入区域，开始计时
            is_in_zone_ = true;
            timer_start_ = std::chrono::steady_clock::now();
            if (node_) {
                RCLCPP_INFO(node_->get_logger(), "进入区域2黄色点范围，开始计时 (距离=%.2fm)", distance);
            }
        }
        else if (!currently_in_zone && is_in_zone_) {
            // 离开区域，重置计时器
            is_in_zone_ = false;
            timer_start_ = std::chrono::steady_clock::time_point::min();
            if (node_) {
                RCLCPP_INFO(node_->get_logger(), "离开区域2黄色点范围，计时器重置");
            }
        }

        // 检查计时器
        if (is_in_zone_ && timer_start_ != std::chrono::steady_clock::time_point::min()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - timer_start_);

            if (elapsed.count() >= stay_duration_) {
                if (node_) {
                    RCLCPP_INFO(node_->get_logger(), "在区域2黄色点停留超过%.1f秒，准备前往区域3",
                               stay_duration_);
                }
                // 重置计时器
                is_in_zone_ = false;
                timer_start_ = std::chrono::steady_clock::time_point::min();
                return BT::NodeStatus::SUCCESS;
            } else {
                if (node_) {
                    double remaining = stay_duration_ - elapsed.count();
                    RCLCPP_DEBUG(node_->get_logger(), "区域2黄色点停留中: %.1f秒 / %.1f秒 (剩余%.1f秒)",
                               static_cast<double>(elapsed.count()), stay_duration_, remaining);
                }
            }
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    double stay_duration_;
    double tolerance_;
    bool is_in_zone_;
    std::chrono::steady_clock::time_point timer_start_;
};

} // namespace rm2026_decision