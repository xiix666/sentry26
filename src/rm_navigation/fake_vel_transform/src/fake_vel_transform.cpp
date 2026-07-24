// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"
#include <chrono>
#include <mutex>
#include <algorithm>
#include <cmath>
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.2;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "fake_base_link");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_safe_cmd_vel_topic", "cmd_vel_safe");
    // this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  // this->declare_parameter<float>("init_spin_speed", 0.0);
  // this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("output_safe_cmd_vel_topic", output_safe_cmd_vel_topic_);

  this->declare_parameter<bool>(
  "enable_linear_deceleration_limit",
  true);

  this->declare_parameter<double>(
    "max_linear_deceleration",
    6.0);

  this->get_parameter(
    "enable_linear_deceleration_limit",
    enable_linear_deceleration_limit_);

  this->get_parameter(
    "max_linear_deceleration",
    max_linear_deceleration_);

  max_linear_deceleration_ =
    std::max(
      EPSILON,
      max_linear_deceleration_);
  // this->get_parameter("local_plan_topic", local_plan_topic_);
  // this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  // this->get_parameter("init_spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
 
  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);
  cmd_vel_safe_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_safe_cmd_vel_topic_, 1);
  // cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
  //   cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  // 初始化缓存
  last_cmd_vel_ =
    std::make_shared<geometry_msgs::msg::Twist>();

  limited_cmd_vel_ =
    geometry_msgs::msg::Twist();

  const auto now =
    this->get_clock()->now();

  last_cmd_vel_time_ = now;
  last_velocity_limit_time_ = now;
  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransformAndSafeVel, this));

}

void FakeVelTransform::odometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const double yaw =
    tf2::getYaw(
      msg->pose.pose.orientation);

  std::lock_guard<std::mutex> lock(
    cmd_vel_mutex_);

  current_robot_base_angle_ = yaw;
}

void FakeVelTransform::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(
    cmd_vel_mutex_);

  // 只缓存控制器给出的目标速度。
  *last_cmd_vel_ = *msg;

  last_cmd_vel_time_ =
    this->get_clock()->now();
}
geometry_msgs::msg::Twist
FakeVelTransform::applyLinearDecelerationLimit(
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target,
  double dt) const
{
  // 先让所有分量直接跟随目标。
  // 因此angular.z不会进行任何平滑处理。
  geometry_msgs::msg::Twist output = target;

  if (!enable_linear_deceleration_limit_) {
    return output;
  }

  const double current_vx =
    current.linear.x;

  const double current_vy =
    current.linear.y;

  const double target_vx =
    target.linear.x;

  const double target_vy =
    target.linear.y;

  const double current_speed =
    std::hypot(
      current_vx,
      current_vy);

  const double target_speed =
    std::hypot(
      target_vx,
      target_vy);

  // 目标速度没有减小，则直接跟随。
  // 因此这里只限制减速度，不限制加速度。
  if (
    target_speed >=
    current_speed - EPSILON)
  {
    return output;
  }

  const double delta_vx =
    target_vx -
    current_vx;

  const double delta_vy =
    target_vy -
    current_vy;

  const double delta_velocity =
    std::hypot(
      delta_vx,
      delta_vy);

  const double max_delta_velocity =
    max_linear_deceleration_ *
    dt;

  // 本次允许的变化量足够达到目标值。
  if (
    delta_velocity <=
    max_delta_velocity ||
    delta_velocity <= EPSILON)
  {
    return output;
  }

  const double scale =
    max_delta_velocity /
    delta_velocity;

  output.linear.x =
    current_vx +
    delta_vx * scale;

  output.linear.y =
    current_vy +
    delta_vy * scale;

  // angular.z仍然保持target.angular.z，
  // 不经过任何限速和平滑。
  return output;
}
// void FakeVelTransform::publishTransform()
// {
//   geometry_msgs::msg::TransformStamped t;
//   t.header.stamp = this->get_clock()->now();
//   t.header.frame_id = robot_base_frame_;
//   t.child_frame_id = fake_robot_base_frame_;
//   tf2::Quaternion q;
//   q.setRPY(0, 0, -current_robot_base_angle_);
//   t.transform.rotation = tf2::toMsg(q);
//   tf_broadcaster_->sendTransform(t);
// }
void FakeVelTransform::publishTransformAndSafeVel()
{
  const auto now =
    this->get_clock()->now();

  geometry_msgs::msg::Twist target_vel;
  geometry_msgs::msg::Twist current_limited_vel;

  double current_yaw = 0.0;
  double dt = 0.02;

  {
    std::lock_guard<std::mutex> lock(
      cmd_vel_mutex_);

    dt =
      (now -
      last_velocity_limit_time_).seconds();

    last_velocity_limit_time_ = now;

    // 防止定时器阻塞或时间跳变造成单次速度变化过大。
    dt = std::clamp(
      dt,
      0.001,
      0.1);

    const double cmd_age =
      (now -
      last_cmd_vel_time_).seconds();

    if (cmd_age < CONTROLLER_TIMEOUT) {
      target_vel =
        *last_cmd_vel_;
    } else {
      target_vel =
        geometry_msgs::msg::Twist();
    }

    current_limited_vel =
      limited_cmd_vel_;

    current_yaw =
      current_robot_base_angle_;
  }

  geometry_msgs::msg::Twist safe_vel =
    applyLinearDecelerationLimit(
      current_limited_vel,
      target_vel,
      dt);

  {
    std::lock_guard<std::mutex> lock(
      cmd_vel_mutex_);

    limited_cmd_vel_ =
      safe_vel;
  }

  // ============================================================
  // 1. 发布TF变换
  // ============================================================
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = now;
  t.header.frame_id =
    robot_base_frame_;

  t.child_frame_id =
    fake_robot_base_frame_;

  tf2::Quaternion q;

  q.setRPY(
    0.0,
    0.0,
    -current_yaw);

  t.transform.rotation =
    tf2::toMsg(q);

  tf_broadcaster_->sendTransform(t);

  // ============================================================
  // 2. 发布未变换的安全速度
  // ============================================================
  cmd_vel_safe_pub_->publish(
    safe_vel);

  // ============================================================
  // 3. 转换到底盘坐标系并发布
  // ============================================================
  const auto chassis_vel =
    transformVelocity(
      safe_vel,
      current_yaw);

  cmd_vel_chassis_pub_->publish(
    chassis_vel);
}
geometry_msgs::msg::Twist
FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist,
  double yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;

  aft_tf_vel.angular.z =
    twist.angular.z;

  aft_tf_vel.linear.x =
    twist.linear.x *
      std::cos(yaw_diff) +
    twist.linear.y *
      std::sin(yaw_diff);

  aft_tf_vel.linear.y =
    -twist.linear.x *
      std::sin(yaw_diff) +
    twist.linear.y *
      std::cos(yaw_diff);

  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
