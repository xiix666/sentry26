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
 * @brief 区域3两点随机巡航的异步行为节点
 * 在两个点之间随机巡航，到达一点后切换到另一目标点
 */
class TestQif : public BT::StatefulActionNode
{
public:
    TestQif(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , node_(nullptr)
        , navigation_started_(false)
        , current_target_index_(-1)
        , is_waiting_(false)
        , rng_(std::random_device{}())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("point1", "第一个巡逻点"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("point2", "第二个巡逻点"),
            BT::InputPort<double>("timeout", 15.0, "单点导航超时时间(秒)"),
            BT::InputPort<double>("tolerance", 0.1, "到达目标点容忍距离(米)"),
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2节点实例"),
            BT::InputPort<float>("current_pose_x", "@current_pose_x", "当前机器人X坐标(米)"),
            BT::InputPort<float>("current_pose_y", "@current_pose_y", "当前机器人Y坐标(米)"),
            BT::InputPort<int>("center_status", "@center_status", "中心状态"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_goal", "@nav_goal", "导航目标(用于距离判断, inout)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goalOut", "导航目标"),
            BT::OutputPort<bool>("nav_goal_requested", "导航目标请求"),
        };
    }

    BT::NodeStatus onStart() override
    {
        // 获取ROS2节点
        auto node_input = getInput<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_input) {
            RCLCPP_ERROR(rclcpp::get_logger("TestQif"), "无法获取ROS2节点实例");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        // 获取两个巡逻点
        auto point1 = getInput<geometry_msgs::msg::PoseStamped>("point1");
        auto point2 = getInput<geometry_msgs::msg::PoseStamped>("point2");

        if (!point1 || !point2) {
            RCLCPP_ERROR(node_->get_logger(), "无法获取巡逻点");
            return BT::NodeStatus::FAILURE;
        }

        patrol_points_[0] = point1.value();
        patrol_points_[1] = point2.value();

        // 获取容忍距离
        auto tolerance = getInput<double>("tolerance");
        tolerance_ = tolerance.value_or(0.2);

        // 随机选择第一个目标点
        selectRandomTarget();
        navigateToCurrentTarget();

        RCLCPP_INFO(node_->get_logger(), "开始区域3两点随机巡航，第一个目标点: %d", current_target_index_);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

        // 处于等待状态时，检查等待时间是否结束
        if (is_waiting_) {
            if (std::chrono::steady_clock::now() >= wait_end_point_) {
                RCLCPP_INFO(node_->get_logger(), "停顿结束，切换到下一个目标点");
                is_waiting_ = false;
                selectRandomTarget();
                navigateToCurrentTarget();
            }
            return BT::NodeStatus::RUNNING;
        }

        // 检查是否超时
        if (std::chrono::steady_clock::now() > timeout_point_) {
            RCLCPP_WARN(node_->get_logger(), "导航到巡逻点超时，切换目标点");
            selectRandomTarget();
            navigateToCurrentTarget();
            return BT::NodeStatus::RUNNING;
        }

        // 仅按当前点到目标点距离判断到达
        if (checkDistanceToTarget()) {
            RCLCPP_INFO(node_->get_logger(), "已到达巡逻点%d，开始停顿1秒", current_target_index_);
            is_waiting_ = true;
            wait_end_point_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            return BT::NodeStatus::RUNNING;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "巡航被中断");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("TestQif"), "巡航被中断（节点未初始化）");
        }
        navigation_started_ = false;
        current_target_index_ = -1;
        is_waiting_ = false;
    }

private:
    void selectRandomTarget()
    {
        std::uniform_int_distribution<int> dist(0, 1);
        int new_target;

        // 避免连续选择同一个点（两点巡逻时直接切换）
        do {
            new_target = dist(rng_);
        } while (new_target == current_target_index_ && patrol_points_count_ > 1);

        current_target_index_ = new_target;

        if (node_) {
            RCLCPP_DEBUG(node_->get_logger(), "选择巡逻点: %d", current_target_index_);
        }
    }

    void navigateToCurrentTarget()
    {
        if (!node_) {
            return;
        }

        if (current_target_index_ < 0 || current_target_index_ >= 2) {
            RCLCPP_ERROR(node_->get_logger(), "无效的巡逻点索引: %d", current_target_index_);
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

        RCLCPP_INFO(node_->get_logger(), "导航到巡逻点%d (x=%.2f, y=%.2f), 容忍距离=%.2fm",
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

        if (current_target_index_ < 0 || current_target_index_ >= 2) {
            return false;
        }

        // 获取当前机器人位置
        auto current_x_input = getInput<float>("current_pose_x");
        auto current_y_input = getInput<float>("current_pose_y");

        if (!current_x_input || !current_y_input) {
            RCLCPP_WARN(node_->get_logger(), "无法获取机器人当前位置");
            return false;
        }

        float current_x = current_x_input.value();
        float current_y = current_y_input.value();

        // 从黑板变量 nav_goal 读取目标位姿
        auto nav_goal_input = getInput<geometry_msgs::msg::PoseStamped>("nav_goal");
        if (!nav_goal_input) {
            RCLCPP_WARN(node_->get_logger(), "无法从黑板获取 nav_goal");
            return false;
        }
        float target_x = nav_goal_input.value().pose.position.x;
        float target_y = nav_goal_input.value().pose.position.y;

        // 计算距离
        float dx = target_x - current_x;
        float dy = target_y - current_y;
        float distance = std::sqrt(dx * dx + dy * dy);

        RCLCPP_DEBUG(node_->get_logger(), "距离检查: 当前位置(%.2f, %.2f), 目标位置(%.2f, %.2f), 距离=%.2fm, 容忍距离=%.2fm, 结果=%s",
                    current_x, current_y, target_x, target_y, distance, tolerance_, (distance <= tolerance_) ? "通过" : "不通过");

        return distance <= tolerance_;
    }

    rclcpp::Node::SharedPtr node_;
    std::array<geometry_msgs::msg::PoseStamped, 2> patrol_points_;
    std::chrono::steady_clock::time_point timeout_point_;
    std::chrono::steady_clock::time_point wait_end_point_;
    double tolerance_;
    bool navigation_started_;
    int current_target_index_;
    bool is_waiting_;
    std::mt19937 rng_;
    static constexpr int patrol_points_count_ = 2;
};

} // namespace rm2026_decision
