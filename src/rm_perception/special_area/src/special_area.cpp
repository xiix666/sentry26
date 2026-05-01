#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include <cmath>

class SpecialArea : public rclcpp::Node
{
public:
    SpecialArea(const rclcpp::NodeOptions &node_options) : Node("special_area", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "SpecialArea node has been started.");

        // 原始区域参数
        this->declare_parameter("left_top_x", 8.641380);
        this->declare_parameter("left_top_y", 2.241885);
        this->declare_parameter("right_bottom_x", 6.396358);
        this->declare_parameter("right_bottom_y", -1.603606);

        this->get_parameter("left_top_x", left_top_x);
        this->get_parameter("left_top_y", left_top_y);
        this->get_parameter("right_bottom_x", right_bottom_x);
        this->get_parameter("right_bottom_y", right_bottom_y);

        // 原始小区域
        min_x = std::min(left_top_x, right_bottom_x);
        max_x = std::max(left_top_x, right_bottom_x);
        min_y = std::min(left_top_y, right_bottom_y);
        max_y = std::max(left_top_y, right_bottom_y);

        // 滞回大区域（向外拓展0.2）
        hysteresis_min_x = min_x - 0.2;
        hysteresis_max_x = max_x + 0.2;
        hysteresis_min_y = min_y - 0.2;
        hysteresis_max_y = max_y + 0.2;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        spin_pub_ = this->create_publisher<std_msgs::msg::Int32>("/spin_status", 10);
        gimbal_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gimbal_angle", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpecialArea::checkRobotPosition, this)
        );

        gimbal_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpecialArea::publishGimbalAngle, this)
        );

        inside_active = false;
        current_gimbal_angle_ = 0.0;
    }

private:

    double left_top_x, left_top_y, right_bottom_x, right_bottom_y;
    double min_x, min_y, max_x, max_y;
    double hysteresis_min_x, hysteresis_max_x, hysteresis_min_y, hysteresis_max_y;

    bool inside_active;
    double current_gimbal_angle_;  
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr spin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_angle_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr gimbal_timer_;

    double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void checkRobotPosition()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        geometry_msgs::msg::TransformStamped fake_tf;

        try {
            transformStamped = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
            fake_tf = tf_buffer_->lookupTransform("base_link", "fake_base_link", rclcpp::Time(0));

            double robot_x = transformStamped.transform.translation.x;
            double robot_y = transformStamped.transform.translation.y;
            double rel_yaw = tf2::getYaw(fake_tf.transform.rotation);
            double gimbal_angle = normalizeAngle(rel_yaw + M_PI_2);
            gimbal_angle = gimbal_angle * 180 / M_PI;

            // 保存最新角度
            current_gimbal_angle_ = gimbal_angle;

            // 滞回判断
            bool now_inside_small = (robot_x > min_x && robot_x < max_x &&
                                    robot_y > min_y && robot_y < max_y);
            bool now_inside_large = (robot_x > hysteresis_min_x && robot_x < hysteresis_max_x &&
                                    robot_y > hysteresis_min_y && robot_y < hysteresis_max_y);

            if (now_inside_small) {
                inside_active = true;
            } else if (!now_inside_large) {
                inside_active = false;
            }

            std_msgs::msg::Int32 msg;
            msg.data = inside_active ? 0 : 1;
            spin_pub_->publish(msg);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    void publishGimbalAngle()
    {
        if (inside_active) {
            std_msgs::msg::Float32 angle_msg;
            angle_msg.data = current_gimbal_angle_;
            gimbal_angle_pub_->publish(angle_msg);
        }
    }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SpecialArea)
