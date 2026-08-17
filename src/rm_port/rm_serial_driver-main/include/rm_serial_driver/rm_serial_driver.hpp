// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "std_msgs/msg/float32.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "rm_interfaces/msg/receive_msg.hpp"
#include "rm_interfaces/msg/lidar_msg.hpp"
#include "rm_interfaces/msg/angle_msg.hpp"
#include "rm_interfaces/msg/shoot_msg.hpp"
#include "rm_interfaces/msg/nav_msg.hpp"
#include "rm_interfaces/msg/special_msg.hpp"
#include "rm_interfaces/msg/send_to_lidar_msg.hpp"
#include "rm_interfaces/msg/receive_llc.hpp"
#include "rm_interfaces/msg/status_msg.hpp"


namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void angleData(const geometry_msgs::msg::Point32::SharedPtr msg);

  void shootData(const rm_interfaces::msg::ShootMsg::SharedPtr msg);


  void navData(const rm_interfaces::msg::NavMsg::SharedPtr msg);

  void specialNumberData(const rm_interfaces::msg::SpecialMsg::SharedPtr msg);

  void sendEnemyPosesData(const rm_interfaces::msg::SendToLidarMsg::SharedPtr msg);

  void stanceData(const rm_interfaces::msg::StatusMsg::SharedPtr msg);
  
  void cmdData(const geometry_msgs::msg::Twist::SharedPtr msg);

  void cmd_save_Data(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  void saveData(const std_msgs::msg::Int32::SharedPtr msg);

  void spinData(const std_msgs::msg::Int32::SharedPtr msg);

  void specialAreaAngleData(const std_msgs::msg::Float32::SharedPtr msg);

  void areaStatusData(const std_msgs::msg::Int32::SharedPtr msg);

  void outpostData(const std_msgs::msg::Int32::SharedPtr msg);

  void go_qifuData(const std_msgs::msg::Int32::SharedPtr msg);
  
  void aimEnableNavData(const std_msgs::msg::Int32::SharedPtr msg);

  void sendData();

  void timer_callback();

  void reopenPort();
  
  void gimbalAngleData(const std_msgs::msg::Float32::SharedPtr msg);

  void updateNavForceArea();
  std::atomic<float> speed_x{0.0f};
  std::atomic<float> speed_y{0.0f};
  std::atomic<float> speed_save_x{0.0f};
  std::atomic<float> speed_save_y{0.0f};
  std::atomic<int> self_save_status{0};
  std::atomic<float> angle{0.0f};
  std::atomic<float> pitch{0.0f};
  std::atomic<uint8_t> shoot_mode{0};
  std::atomic<uint8_t> special_number{0};  
  std::atomic<uint8_t> spin_enable{1};
  std::atomic<uint8_t> nav_enable{1};
  std::atomic<uint8_t> sentry_stance{3};
  std::atomic<uint8_t> area_status{0};
  std::atomic<uint8_t> outpost_enable{0};
  std::atomic<uint8_t> go_qifu{0};
  std::atomic<float> area_angle{0.0f};
  std::array<std::array<float, 2>, 5> send_enemy_poses_;
  std::mutex send_enemy_poses_mutex_;
  std::atomic<float> gimbal_angle{0.0f};
  std::atomic_bool aim_force_nav_enable_{false};
  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr nav_force_area_timer_;

  // true 时无条件覆盖串口 nav_enable 为 1
  std::atomic_bool force_nav_enable_{false};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aim_enable_nav_sub_;
  // double timestamp_offset_ = 0;

  rclcpp::Publisher<rm_interfaces::msg::ReceiveMsg>::SharedPtr receive_pub_;
  rclcpp::Publisher<rm_interfaces::msg::ReceiveLLC>::SharedPtr receiveLLC_pub_;
  rclcpp::Publisher<rm_interfaces::msg::LidarMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_speed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr special_area_angle_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr area_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr decision_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr angle_sub_;
  rclcpp::Subscription<rm_interfaces::msg::ShootMsg>::SharedPtr shoot_mode_sub_;
  rclcpp::Subscription<rm_interfaces::msg::NavMsg>::SharedPtr nav_enable_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SpecialMsg>::SharedPtr special_number_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SendToLidarMsg>::SharedPtr send_enemy_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_save_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr save_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spin_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr outpost_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr go_qifu_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_angle_sub_;
  rclcpp::Subscription<rm_interfaces::msg::StatusMsg>::SharedPtr stance_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_reset;

  std::thread receive_thread_;
  uint32_t baud_rate{};
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
