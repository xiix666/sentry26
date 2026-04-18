#ifndef ANGLE_PUB_HPP_
#define ANGLE_PUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <angles/angles.h>

#include "rm_interfaces/msg/shoot_msg.hpp"
#include "rm_interfaces/msg/special_msg.hpp"
#include "rm_interfaces/msg/angle_pub_msg.hpp"

class AnglePub : public rclcpp::Node
{
public:
    AnglePub();

private:
    // TF相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS接口
    rclcpp::Publisher<rm_interfaces::msg::AnglePubMsg>::SharedPtr angle_sp_pub_;
    rclcpp::Subscription<rm_interfaces::msg::ShootMsg>::SharedPtr shoot_mode_sub_;
    rclcpp::Subscription<rm_interfaces::msg::SpecialMsg>::SharedPtr special_number_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态变量
    geometry_msgs::msg::Twist latest_cmd_vel_;
    double latest_velocity_direction_;
    double send_angle_;
    std::mutex angle_mutex_;
    std::mutex cmd_vel_mutex_;
    uint8_t current_shoot_mode_;
    uint8_t current_special_number_;
    double yaw_energy_;
    double yaw_tunnel_forward_;
    double yaw_tunnel_backward_;

    // 辅助函数
    double getYaw();
    double normalizeAngle(double angle);

    // 回调函数
    void shootModeCallback(const rm_interfaces::msg::ShootMsg::SharedPtr msg);
    void specialNumberCallback(const rm_interfaces::msg::SpecialMsg::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timerCallback();
};

#endif // ANGLE_PUB_HPP_