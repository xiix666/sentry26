#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fake_vel_transform
{

class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);

  void odometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  void localPlanCallback(
    const nav_msgs::msg::Path::ConstSharedPtr & msg);

  void cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg);

  void cmdSpinCallback(
    example_interfaces::msg::Float32::SharedPtr msg);

  void publishTransformAndSafeVel();

  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist & twist,
    double yaw_diff);

  geometry_msgs::msg::Twist applyLinearDecelerationLimit(
    const geometry_msgs::msg::Twist & current,
    const geometry_msgs::msg::Twist & target,
    double dt) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;

  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Path>;

  std::unique_ptr<
    message_filters::Synchronizer<SyncPolicy>>
    sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_chassis_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_safe_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string output_safe_cmd_vel_topic_;

  float spin_speed_{0.0F};

  std::mutex cmd_vel_mutex_;

  double current_robot_base_angle_{0.0};

  geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;

  geometry_msgs::msg::Twist limited_cmd_vel_;

  rclcpp::Time last_cmd_vel_time_;

  rclcpp::Time last_velocity_limit_time_;

  bool enable_linear_deceleration_limit_{true};

  double max_linear_deceleration_{6.0};

  rclcpp::Time last_controller_activate_time_;

  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
};

}

#endif
