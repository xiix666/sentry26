#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <atomic>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/int32.hpp"

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

constexpr double ANGLE_EPSILON = 1e-6; // 浮点数比较阈值

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  nav_force_area_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&RMSerialDriver::updateNavForceArea, this));
  // Create Publisher
  receive_pub_ = this->create_publisher<rm_interfaces::msg::ReceiveMsg>("/receive_pack",rclcpp::QoS(10));
  receiveLLC_pub_ = this->create_publisher<rm_interfaces::msg::ReceiveLLC>("/receiveLLC_pack",rclcpp::QoS(10));
  lidar_pub_ = this->create_publisher<rm_interfaces::msg::LidarMsg>("/lidar_pack",rclcpp::QoS(10));
  decision_pub_ = this->create_publisher<std_msgs::msg::Int32>("/decision_change", rclcpp::QoS(10));
  // goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_op",10);
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_op",10);
  pub_speed_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/self_speed", rclcpp::QoS(10));
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      RCLCPP_INFO(get_logger(), "Serial port opened successfully!");
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // Create Subscription
  special_area_angle_pub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/special_area_angle", rclcpp::QoS(10),
    std::bind(&RMSerialDriver::specialAreaAngleData, this, std::placeholders::_1));
  area_status_pub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/area_status", rclcpp::QoS(10),
    std::bind(&RMSerialDriver::areaStatusData, this, std::placeholders::_1));
  angle_sub_ = this->create_subscription<geometry_msgs::msg::Point32>(
    "/angle_pack", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::angleData, this, std::placeholders::_1));
  shoot_mode_sub_ = this->create_subscription<rm_interfaces::msg::ShootMsg>(
    "/shoot_pack", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::shootData, this, std::placeholders::_1));
  nav_enable_sub_ = this->create_subscription<rm_interfaces::msg::NavMsg>(
    "/nav_pack", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::navData, this, std::placeholders::_1));
  special_number_sub_ = this->create_subscription<rm_interfaces::msg::SpecialMsg>(
    "/special_pack", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::specialNumberData, this, std::placeholders::_1));
  send_enemy_poses_sub_ = this->create_subscription<rm_interfaces::msg::SendToLidarMsg>(
    "/send_to_lidar_pack", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendEnemyPosesData, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_base", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::cmdData, this, std::placeholders::_1));
  cmd_vel_save_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_save", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::cmd_save_Data, this, std::placeholders::_1));
  save_sub = this->create_subscription<std_msgs::msg::Int32>(
    "/self_save_status", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::saveData, this, std::placeholders::_1));
  spin_sub = this->create_subscription<std_msgs::msg::Int32>(
    "/spin_status", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::spinData, this, std::placeholders::_1));
  gimbal_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/gimbal_angle", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::gimbalAngleData, this, std::placeholders::_1));
  stance_sub_ = this->create_subscription<rm_interfaces::msg::StatusMsg>(
    "/status_pack", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::stanceData, this, std::placeholders::_1));
  // Create a timer to run sendData at 100 Hz
  outpost_sub = this->create_subscription<std_msgs::msg::Int32>(
    "/outpost_lock_lost", 10,
    std::bind(&RMSerialDriver::outpostData, this, std::placeholders::_1));
  go_qifu_sub = this->create_subscription<std_msgs::msg::Int32>(
    "/ally_undulation_pass", 10,
    std::bind(&RMSerialDriver::go_qifuData, this, std::placeholders::_1));
  aim_enable_nav_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/aim_enable_nav",
    rclcpp::QoS(10),
    std::bind(
      &RMSerialDriver::aimEnableNavData,
      this,
      std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),  // 10 ms = 100 Hz
    std::bind(&RMSerialDriver::sendData, this));
  
  // timer_reset = this->create_wall_timer(
  //   std::chrono::milliseconds(10000),  // 0.1Hz
  //   std::bind(&RMSerialDriver::timer_callback, this));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data_a;
  std::vector<uint8_t> data_ul;
  std::vector<uint8_t> data_uc;
  std::vector<uint8_t> data_b;
  // std::vector<uint8_t> data;
  data_a.reserve(sizeof(ReceivePacketA));
  // data.reserve(sizeof(ReceivePacket));
  // data_ul.reserve(sizeof(ReceivePacketUL));
  data_uc.reserve(sizeof(ReceivePacketUC));
  data_b.reserve(sizeof(ReceivePacketB));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);
      // std::cout << "sizeof:" << sizeof(ReceivePacketUL)  <<std::endl;
      if (header[0] == 0xA5) {
        data_a.resize(sizeof(ReceivePacketA) - 1);
        serial_driver_->port()->receive(data_a);

        data_a.insert(data_a.begin(), header[0]);

        // std::cout<< "A5" << std::endl;
        // std::cout << "Size of ReceivePacketA: " << sizeof(ReceivePacketA) << " bytes" << std::endl;

        // for(long unsigned int i=0;i<data_a.size();i++){
        //   std::cout<< std::hex<<std::setw(2)<<std::setfill('0') << static_cast<int>(data_a[i]) <<"  ";
        // }
        // std::cout<<std::endl;

        ReceivePacketA packet = fromVectorA(data_a);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          // rclcpp::Time now_time = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          rm_interfaces::msg::ReceiveMsg pub_pack;
          // pub_pack.header.stamp = now_time;
          pub_pack.game_progress = packet.game_progress;
          pub_pack.stage_remain_time = packet.stage_remain_time;
          pub_pack.red_blue = packet.red_blue;
          pub_pack.self_1_hp = packet.self_1_hp;
          pub_pack.self_2_hp = packet.self_2_hp;
          pub_pack.self_3_hp = packet.self_3_hp;
          pub_pack.self_4_hp = packet.self_4_hp;
          pub_pack.self_7_hp = packet.self_7_hp;
          pub_pack.self_outpost_hp = packet.self_outpost_hp;
          pub_pack.self_base_hp = packet.self_base_hp;
          pub_pack.bullets_allowance = packet.bullets_allowance;
          pub_pack.chassis_power = packet.chassis_power;
          pub_pack.self_small_energy = packet.self_small_energy;
          pub_pack.self_big_energy = packet.self_big_energy;
          pub_pack.total_bullets = packet.total_bullets;
          pub_pack.post_1_hp = packet.post_1_hp;
          pub_pack.post_2_hp = packet.post_2_hp;
          pub_pack.post_3_hp = packet.post_3_hp;
          pub_pack.post_4_hp = packet.post_4_hp;
          pub_pack.post_7_hp = packet.post_7_hp;
          pub_pack.post_outpost_hp = packet.post_outpost_hp;
          pub_pack.post_base_hp = packet.post_base_hp;
          pub_pack.dart_time = packet.dart_time;
          pub_pack.dart_type = packet.dart_type;
          pub_pack.remain_gold = packet.remain_gold;
          pub_pack.operator_x = packet.operator_x;
          pub_pack.operator_y = packet.operator_y;
          
          pub_pack.heat_flag = packet.heat_flag;

          receive_pub_->publish(pub_pack);
        } 
        else {
          RCLCPP_ERROR(get_logger(), "A5 CRC error!");
        }
      }
      else if (header[0] == 0xA3){
        data_uc.resize(sizeof(ReceivePacketUC) - 1);
        serial_driver_->port()->receive(data_uc);

        data_uc.insert(data_uc.begin(), header[0]);

        // std::cout<< "A5" << std::endl;
        // std::cout << "Size of ReceivePacketA: " << sizeof(ReceivePacketA) << " bytes" << std::endl;

        // for(long unsigned int i=0;i<data_a.size();i++){
        //   std::cout<< std::hex<<std::setw(2)<<std::setfill('0') << static_cast<int>(data_a[i]) <<"  ";
        // }
        // std::cout<<std::endl;

        ReceivePacketUC packet = fromVectoruc(data_uc);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          // rclcpp::Time now_time = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          rm_interfaces::msg::ReceiveLLC pub_pack;
          geometry_msgs::msg::Point pub_speed;
          geometry_msgs::msg::PoseStamped pose_pack;
          std_msgs::msg::Int32 decision_pack;
          pub_pack.game_progress = packet.game_progress;
          pub_pack.stage_remain_time = packet.stage_remain_time;
          pub_pack.red_blue = packet.red_blue;
          pub_pack.self_hero_hp = packet.self_hero_hp;
          pub_pack.self_engineer_hp = packet.self_engineer_hp;
          pub_pack.self_infantry3_hp = packet.self_infantry3_hp;
          pub_pack.self_infantry4_hp = packet.self_infantry4_hp;
          pub_pack.self_sentry_hp = packet.self_sentry_hp;
          pub_pack.self_outpost_hp = packet.self_outpost_hp;
          pub_pack.self_base_hp = packet.self_base_hp;

          pub_pack.enemy_outpost_hp = packet.enemy_outpost_hp;
          pub_pack.self_pose_x = packet.self_pose_x;
          pub_pack.self_pose_y = packet.self_pose_y;
          pub_pack.bullets_allowance = packet.bullets_allowance;
          pub_pack.self_hero_pose_x = packet.self_hero_pose_x;
          pub_pack.self_hero_pose_y = packet.self_hero_pose_y;
          pub_pack.self_engineer_pose_x = packet.self_engineer_pose_x;
          pub_pack.self_engineer_pose_y = packet.self_engineer_pose_y;
          pub_pack.self_infantry3_pose_x = packet.self_infantry3_pose_x;
          pub_pack.self_infantry3_pose_y = packet.self_infantry3_pose_y;
          pub_pack.self_infantry4_pose_x = packet.self_infantry4_pose_x;
          pub_pack.self_infantry4_pose_y = packet.self_infantry4_pose_y;
          // pub_pack.center_status = packet.center_status;
          // pub_pack.center_status = 0;
          pub_pack.remain_gold = packet.remain_gold;
          pub_pack.recovery_buff = packet.recovery_buff;
          pub_pack.defence_buff = packet.defence_buff;
          pub_pack.defence_debuff = packet.defence_debuff;
          pub_pack.attack_buff = packet.attack_buff;
          pub_pack.status_info = packet.status_info;
          pub_pack.enemy_invincible = packet.enemy_invincible;
          receiveLLC_pub_->publish(pub_pack);
          pub_speed.x = packet.self_speed_x;
          pub_speed.y = packet.self_speed_y;
          pub_speed.z = 0.0;
          pub_speed_pub_->publish(pub_speed);
          pose_pack.pose.position.x = packet.operator_x;
          pose_pack.pose.position.y = packet.operator_y;
          decision_pack.data = packet.decision_node;
          decision_pub_->publish(decision_pack);
          goal_pub_->publish(pose_pack);
      }
        else {
        RCLCPP_ERROR(get_logger(), "A4 CRC error!");
        reopenPort();
      }
    }
      else if (header[0] == 0xA6) {
        data_b.resize(sizeof(ReceivePacketB) - 1);
        serial_driver_->port()->receive(data_b);

        data_b.insert(data_b.begin(), header[0]);

        // std::cout<< "A6" << std::endl;
        // std::cout << "Size of ReceivePacketB: " << sizeof(ReceivePacketB) << " bytes" << std::endl;

        // for(long unsigned int i=0;i<data_b.size();i++){
        //   std::cout<< std::hex<<std::setw(2)<<std::setfill('0') << static_cast<int>(data_b[i]) <<"  ";
        // }
        // std::cout<<std::endl;

        ReceivePacketB packet = fromVectorB(data_b);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {

          rm_interfaces::msg::LidarMsg lidar_pack;

          for (size_t i = 0; i < packet.enemy_poses.size() && i < 5; ++i) {
            lidar_pack.enemy_poses[i].pose.position.x = packet.enemy_poses[i][0] * 0.01;
            lidar_pack.enemy_poses[i].pose.position.y = packet.enemy_poses[i][1] * 0.01;
            lidar_pack.enemy_poses[i].pose.position.z = 0.0;  
            lidar_pack.enemy_poses[i].pose.orientation.x = 0.0;  
            lidar_pack.enemy_poses[i].pose.orientation.y = 0.0;
            lidar_pack.enemy_poses[i].pose.orientation.z = 0.0;
            lidar_pack.enemy_poses[i].pose.orientation.w = 1.0;
            lidar_pack.enemy_poses[i].header.frame_id = "map";  
            lidar_pack.enemy_poses[i].header.stamp = this->get_clock()->now();  
            lidar_pack.enemy_speed_x[i] = packet.enemy_speed_xy[i][0]*0.01;
            lidar_pack.enemy_speed_y[i] = packet.enemy_speed_xy[i][1]*0.01;
          }

          lidar_pub_->publish(lidar_pack);
        } 
        else {
          RCLCPP_ERROR(get_logger(), "A6 CRC error!");
        }
      }
else {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, 
                      "Invalid header detected: 0x%02X, printing all following bytes", 
                      static_cast<uint8_t>(header[0]));
  
  std::vector<uint8_t> all_invalid_data;
  all_invalid_data.push_back(header[0]); // 先加入无效头字节

  // 定义最大读取后续字节数（可按需调整，如32/64/128，建议64）
  const size_t MAX_FOLLOW_BYTES = 41;
  uint8_t temp_byte;
  // 定长读取后续字节，循环MAX_FOLLOW_BYTES次
  for (size_t i = 0; i < MAX_FOLLOW_BYTES; ++i) {
    try {
      // 修正receive()参数：仅传uint8_t*，匹配接口声明（无需长度参数）
      std::vector<uint8_t> temp_vec(1); // 长度=要接收的字节数
      serial_driver_->port()->receive(temp_vec);
      all_invalid_data.push_back(temp_byte);
    } catch (const std::exception& e) {
      // 捕获读取异常（如缓冲区无数据），立即终止循环，避免阻塞
      break;
    }
  }

  std::cout << "┌──────────────── Invalid Serial Data ────────────────┐" << std::endl;
  std::cout << "│ Total bytes: " << std::setw(3) << all_invalid_data.size() << " | Invalid header: 0x" 
            << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(header[0]) << std::dec << " │" << std::endl;
  std::cout << "│ Hex content:  ";
  for (size_t i = 0; i < all_invalid_data.size(); ++i) {
    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(all_invalid_data[i]);
    if (i != all_invalid_data.size() - 1) std::cout << " ";
  }
  std::cout << std::dec << " │" << std::endl;
  std::cout << "└─────────────────────────────────────────────────────┘" << std::endl;
  reopenPort();
}

    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::angleData(const geometry_msgs::msg::Point32::SharedPtr msg)
{
    angle = msg->x;
    pitch = msg->y;
}
void RMSerialDriver::stanceData(const rm_interfaces::msg::StatusMsg::SharedPtr msg)
{
    sentry_stance = msg->sentry_stance;
}
void RMSerialDriver::shootData(const rm_interfaces::msg::ShootMsg::SharedPtr msg)
{
    shoot_mode = msg->shoot_mode;
}

void RMSerialDriver::navData(const rm_interfaces::msg::NavMsg::SharedPtr msg)
{
    nav_enable = msg->nav_unable;
}

void RMSerialDriver::specialNumberData(const rm_interfaces::msg::SpecialMsg::SharedPtr msg)
{
    special_number = msg->special_number;
}

void RMSerialDriver::gimbalAngleData(const std_msgs::msg::Float32::SharedPtr msg)
{
    gimbal_angle = msg->data;
}
void RMSerialDriver::specialAreaAngleData(const std_msgs::msg::Float32::SharedPtr msg)
{
    area_angle = msg->data;
}
void RMSerialDriver::areaStatusData(const std_msgs::msg::Int32::SharedPtr msg)
{
    area_status = msg->data;
}
void RMSerialDriver::sendEnemyPosesData(const rm_interfaces::msg::SendToLidarMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(send_enemy_poses_mutex_);

    for (size_t i = 0; i < 5; ++i) {
        send_enemy_poses_[i][0] = msg->enemy_poses_x_to_lidar[i];
        send_enemy_poses_[i][1] = msg->enemy_poses_y_to_lidar[i];
    }
}

void RMSerialDriver::cmdData(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    speed_x = float(msg->linear.x);
    speed_y = float(msg->linear.y);
}

void RMSerialDriver::cmd_save_Data(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    speed_save_x = float(msg->linear.x);
    speed_save_y = float(msg->linear.y);
}
void RMSerialDriver::saveData(const std_msgs::msg::Int32::SharedPtr msg)
{
    self_save_status = msg->data;
}
void RMSerialDriver::spinData(const std_msgs::msg::Int32::SharedPtr msg)
{
    spin_enable = msg->data;
}
void RMSerialDriver::outpostData(const std_msgs::msg::Int32::SharedPtr msg)
{
    outpost_enable = msg->data;
}
void RMSerialDriver::go_qifuData(const std_msgs::msg::Int32::SharedPtr msg)
{  
    go_qifu = msg->data;
}
void RMSerialDriver::sendData()
{
  try {
    SendPacket packet;
    if(self_save_status == 1){
      packet.speed_x = speed_save_x;
      packet.speed_y = speed_save_y;
    }
    else{
      packet.speed_x = speed_x;
      packet.speed_y = speed_y;
    }

    // if (area_status == 1) {
    //   packet.angle = gimbal_angle;
    //   packet.pitch = 0.0;
    // } else if (area_status == 2) {
    //   packet.angle = area_angle;
    //   packet.pitch = 0.0;
    // } else {
    //   packet.angle = angle;
    //   packet.pitch = pitch;
    // }
    if (area_status == 1 || area_status == 2 || area_status == 3) {
      packet.angle = gimbal_angle;
      packet.pitch = 0.0;
    } else {
      packet.angle = angle;
      packet.pitch = pitch;
    }
    packet.shoot_mode = shoot_mode;
    packet.area_status = area_status;
    // packet.spin_enable = spin_enable;
    // packet.nav_enable = nav_enable;
    const bool force_nav = force_nav_enable_.load() || aim_force_nav_enable_.load();

    packet.nav_enable = force_nav ? uint8_t{1} : nav_enable.load();
    packet.sentry_stance = sentry_stance;
    packet.outpost_enable = outpost_enable;
    packet.go_qifu = go_qifu;
    {
      std::lock_guard<std::mutex> lock(send_enemy_poses_mutex_);
      packet.send_enemy_poses = send_enemy_poses_;
    }

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data); 
      // RCLCPP_INFO(get_logger(), "%f", packet.speed_x);
  // RCLCPP_INFO(get_logger(), "%f", packet.speed_y);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::timer_callback()
{
  speed_x = 0.0;
  speed_y = 0.0;
  // angle = 0.0;
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "dev/ttyACM0");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "none");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "none");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "1");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}
void RMSerialDriver::aimEnableNavData(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  aim_force_nav_enable_.store(msg->data == 1);
}
void RMSerialDriver::updateNavForceArea()
{
  geometry_msgs::msg::TransformStamped transform;

  try {
    // 查询 base_link 在 map 坐标系下的位置
    transform = tf_buffer_->lookupTransform(
      "map",
      "base_link",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    // TF 短暂中断时保留上一次状态，避免强制标志突然解除
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Cannot lookup map -> base_link: %s",
      ex.what());
    return;
  }

  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;

  constexpr double kMinX = 12.7;
  constexpr double kMaxX = 14.2;
  constexpr double kMinY = 4.0;
  constexpr double kMaxY = 5.0;

  // 退出缓冲
  constexpr double kBoundaryBuffer = 0.1;

  const bool was_inside = force_nav_enable_.load();

  bool now_inside = was_inside;

  if (!was_inside) {
    now_inside =
      x >= kMinX && x <= kMaxX &&
      y >= kMinY && y <= kMaxY;
  } else {
    now_inside =
      x >= kMinX - kBoundaryBuffer &&
      x <= kMaxX + kBoundaryBuffer &&
      y >= kMinY - kBoundaryBuffer &&
      y <= kMaxY + kBoundaryBuffer;
  }

  if (now_inside != was_inside) {
    force_nav_enable_.store(now_inside);

    // RCLCPP_INFO(
    //   get_logger(),
    //   "Nav force area %s: x=%.3f, y=%.3f, serial nav_enable=%d",
    //   now_inside ? "ENTER" : "EXIT",
    //   x,
    //   y,
    //   now_inside ? 1 : static_cast<int>(nav_enable.load()));
  }
}
}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
