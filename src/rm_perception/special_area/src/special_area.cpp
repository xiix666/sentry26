#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

class SpecialArea : public rclcpp::Node
{
public:
    SpecialArea(const rclcpp::NodeOptions &node_options) : Node("special_area", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "SpecialArea node has been started.");

        this->declare_parameter("left_top_x", 0.0);
        this->declare_parameter("left_top_y", 0.0);
        this->declare_parameter("right_bottom_x", 0.0);
        this->declare_parameter("right_bottom_y", 0.0);

        this->get_parameter("left_top_x", left_top_x);
        this->get_parameter("left_top_y", left_top_y);
        this->get_parameter("right_bottom_x", right_bottom_x);
        this->get_parameter("right_bottom_y", right_bottom_y);
        min_x = std::min(left_top_x, right_bottom_x);
        max_x = std::max(left_top_x, right_bottom_x);
        min_y = std::min(left_top_y, right_bottom_y);
        max_y = std::max(left_top_y, right_bottom_y);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        spin_pub_ = this->create_publisher<std_msgs::msg::Int32>("/spin_status", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SpecialArea::checkRobotPosition, this)
        );
    }

private:
    double left_top_x;
    double left_top_y;
    double right_bottom_x;
    double right_bottom_y;
    double min_x,min_y,max_x,max_y;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr spin_pub_;

    void checkRobotPosition()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
            double robot_x = transformStamped.transform.translation.x;
            double robot_y = transformStamped.transform.translation.y;

            if (robot_x > min_x && robot_x < max_x &&
                robot_y > min_y && robot_y < max_y)
            {
                std_msgs::msg::Int32 msg;
                msg.data = 0;
                spin_pub_->publish(msg); 
            }
            else
            {
                std_msgs::msg::Int32 msg;
                msg.data = 1;
                spin_pub_->publish(msg);
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
};
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SpecialArea)