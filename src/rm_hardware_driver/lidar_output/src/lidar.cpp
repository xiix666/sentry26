#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "livox_ros_driver2/msg/custom_msg.hpp"

using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomPoint = livox_ros_driver2::msg::CustomPoint;
using ApproxSyncPolicy =
  message_filters::sync_policies::ApproximateTime<LivoxCustomMsg, LivoxCustomMsg>;

class lidar_output : public rclcpp::Node
{
public:
  lidar_output()
  : Node("lidar_output")
  {
    this->declare_parameter<std::string>("topic1", "livox/lidar_192_168_2_120");
    this->declare_parameter<std::string>("topic2", "livox/lidar_192_168_2_121");
    this->declare_parameter<std::string>("target_frame_id", "lidar_link");

    this->declare_parameter<double>("sync_max_delay", 0.050);
    this->declare_parameter<int>("sync_queue_size", 20);

    this->declare_parameter<bool>("enable_software_rebase", true);
    this->declare_parameter<double>("rebase_alpha", 0.02);
    this->declare_parameter<double>("rebase_max_jump", 0.010);

    this->get_parameter("topic1", topic_1_);
    this->get_parameter("topic2", topic_2_);
    this->get_parameter("target_frame_id", target_frame_id_);
    this->get_parameter("sync_max_delay", sync_max_delay_);
    this->get_parameter("sync_queue_size", sync_queue_size_);
    this->get_parameter("enable_software_rebase", enable_software_rebase_);
    this->get_parameter("rebase_alpha", rebase_alpha_);
    this->get_parameter("rebase_max_jump", rebase_max_jump_sec_);

    sync_queue_size_ = std::max(sync_queue_size_, 2);
    rebase_alpha_ = std::min(std::max(rebase_alpha_, 0.0), 1.0);
    rebase_max_jump_ns_ = static_cast<int64_t>(
      std::llround(std::max(rebase_max_jump_sec_, 0.0) * 1e9));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub1_.subscribe(this, topic_1_);
    sub2_.subscribe(this, topic_2_);

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
      static_cast<uint32_t>(sync_queue_size_), sub1_, sub2_);
    sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_delay_));

    sync_->registerCallback(
      std::bind(
        &lidar_output::sync_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    pub_merged_cloud_ =
      this->create_publisher<LivoxCustomMsg>("/livox/lidar", rclcpp::QoS(20));

    RCLCPP_INFO(
      this->get_logger(),
      "Lidar Output启动: sync_max_delay=%.1f ms, software_rebase=%s, alpha=%.3f",
      sync_max_delay_ * 1000.0,
      enable_software_rebase_ ? "true" : "false",
      rebase_alpha_);
  }

private:

  const float L1_MIN_DIST = 0.1f;
  const float L1_X_MIN = -0.2f;
  const float L1_X_MAX = 0.2f;
  const float L1_Y_MIN = -0.4f;
  const float L1_Y_MAX = 0.1f;
  const float L1_Z_MIN = -0.2f;
  const float L1_Z_MAX = 0.1f;

  const float L2_MIN_DIST = 0.1f;
  const float L2_X_MIN = -0.2f;
  const float L2_X_MAX = 0.2f;
  const float L2_Y_MIN = -0.1f;
  const float L2_Y_MAX = 0.4f;
  const float L2_Z_MIN = -0.2f;
  const float L2_Z_MAX = 0.1f;

  bool filter1(const LivoxCustomPoint & p) const
  {
    const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (d2 < L1_MIN_DIST) {
      return false;
    }
    if (p.x > L1_X_MIN && p.x < L1_X_MAX) {
      return false;
    }
    if (
      p.x > L1_X_MIN && p.x < L1_X_MAX &&
      p.y < L1_Y_MAX && p.y > L1_Y_MIN &&
      p.z < L1_Z_MAX && p.z > L1_Z_MIN)
    {
      return false;
    }
    return true;
  }

  bool filter2(const LivoxCustomPoint & p) const
  {
    const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (d2 < L2_MIN_DIST) {
      return false;
    }
    if (
      p.x > L2_X_MIN && p.x < L2_X_MAX &&
      p.y < L2_Y_MAX && p.y > L2_Y_MIN &&
      p.z < L2_Z_MAX && p.z > L2_Z_MIN)
    {
      return false;
    }
    return true;
  }

  static uint64_t abs_i64(const int64_t value)
  {
    return value >= 0 ?
      static_cast<uint64_t>(value) :
      static_cast<uint64_t>(-(value + 1)) + 1ULL;
  }

  bool update_clock_offset(const int64_t measured_offset_ns)
  {
    if (!clock_offset_initialized_) {
      estimated_clock_offset_ns_ = measured_offset_ns;
      clock_offset_initialized_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "软件rebase初始化: lidar2-lidar1=%.3f ms",
        static_cast<double>(estimated_clock_offset_ns_) / 1e6);
      return true;
    }

    const int64_t innovation_ns =
      measured_offset_ns - estimated_clock_offset_ns_;

    if (abs_i64(innovation_ns) > static_cast<uint64_t>(rebase_max_jump_ns_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "拒绝疑似错配帧: 测量偏移=%.3f ms, 估计偏移=%.3f ms, 跳变=%.3f ms",
        static_cast<double>(measured_offset_ns) / 1e6,
        static_cast<double>(estimated_clock_offset_ns_) / 1e6,
        static_cast<double>(innovation_ns) / 1e6);
      return false;
    }

    estimated_clock_offset_ns_ += static_cast<int64_t>(
      std::llround(rebase_alpha_ * static_cast<double>(innovation_ns)));
    return true;
  }

  static bool rebase_point_time(
    LivoxCustomPoint & point,
    const int64_t corrected_source_base_ns,
    const int64_t output_base_ns)
  {
    if (corrected_source_base_ns < output_base_ns) {
      return false;
    }

    const uint64_t base_delta_ns = static_cast<uint64_t>(
      corrected_source_base_ns - output_base_ns);
    const uint64_t new_offset_ns =
      base_delta_ns + static_cast<uint64_t>(point.offset_time);

    if (new_offset_ns > std::numeric_limits<uint32_t>::max()) {
      return false;
    }

    point.offset_time = static_cast<uint32_t>(new_offset_ns);
    return true;
  }

  void initialize_transform(const LivoxCustomMsg & msg2)
  {
    if (transform_initialized_) {
      return;
    }

    const geometry_msgs::msg::TransformStamped transform_stamp =
      tf_buffer_->lookupTransform(
      target_frame_id_, msg2.header.frame_id, tf2::TimePointZero);

    lidar2_to_target_ = tf2::transformToEigen(transform_stamp.transform);
    transform_initialized_ = true;
  }

  void sync_callback(
    const LivoxCustomMsg::ConstSharedPtr & msg1,
    const LivoxCustomMsg::ConstSharedPtr & msg2)
  {
    try {
      if (msg1->timebase == 0 || msg2->timebase == 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "收到timebase=0的点云，丢弃");
        return;
      }

      initialize_transform(*msg2);

      const int64_t lidar1_base_ns = static_cast<int64_t>(msg1->timebase);
      const int64_t lidar2_raw_base_ns = static_cast<int64_t>(msg2->timebase);
      const int64_t measured_clock_offset_ns =
        lidar2_raw_base_ns - lidar1_base_ns;

      int64_t lidar2_corrected_base_ns = lidar2_raw_base_ns;

      if (enable_software_rebase_) {
        if (!update_clock_offset(measured_clock_offset_ns)) {
          return;
        }

        lidar2_corrected_base_ns =
          lidar2_raw_base_ns - estimated_clock_offset_ns_;
      }

      const int64_t output_base_ns =
        std::min(lidar1_base_ns, lidar2_corrected_base_ns);

      if (output_base_ns < 0) {
        RCLCPP_ERROR(this->get_logger(), "输出timebase为负数，丢弃该帧");
        return;
      }

      LivoxCustomMsg output_msg;
      output_msg.header = msg1->header;
      output_msg.header.frame_id = target_frame_id_;
      output_msg.header.stamp = rclcpp::Time(output_base_ns);
      output_msg.timebase = static_cast<uint64_t>(output_base_ns);
      output_msg.lidar_id = msg1->lidar_id;
      output_msg.points.reserve(msg1->points.size() + msg2->points.size());

      size_t invalid_time_points = 0;

      for (const auto & src_point : msg1->points) {
        if (!filter1(src_point)) {
          continue;
        }

        LivoxCustomPoint dst_point = src_point;
        if (!rebase_point_time(
            dst_point, lidar1_base_ns, output_base_ns))
        {
          ++invalid_time_points;
          continue;
        }
        output_msg.points.push_back(dst_point);
      }

      for (const auto & src_point : msg2->points) {
        if (!filter2(src_point)) {
          continue;
        }

        LivoxCustomPoint dst_point = src_point;
        const Eigen::Vector3d point(src_point.x, src_point.y, src_point.z);
        const Eigen::Vector3d transformed = lidar2_to_target_ * point;
        dst_point.x = static_cast<float>(transformed.x());
        dst_point.y = static_cast<float>(transformed.y());
        dst_point.z = static_cast<float>(transformed.z());

        if (!rebase_point_time(
            dst_point, lidar2_corrected_base_ns, output_base_ns))
        {
          ++invalid_time_points;
          continue;
        }
        output_msg.points.push_back(dst_point);
      }

      if (output_msg.points.empty()) {
        return;
      }

      if (output_msg.points.size() > std::numeric_limits<uint32_t>::max()) {
        RCLCPP_ERROR(this->get_logger(), "合并点数超过uint32_t范围");
        return;
      }

      output_msg.point_num = static_cast<uint32_t>(output_msg.points.size());
      pub_merged_cloud_->publish(output_msg);

      const int64_t corrected_residual_ns =
        lidar2_corrected_base_ns - lidar1_base_ns;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "双雷达rebase: 原始base差=%.3f ms, 估计时钟差=%.3f ms, "
        "校正后起点差=%.3f ms, 点数=%u, 时间异常点=%zu",
        static_cast<double>(measured_clock_offset_ns) / 1e6,
        static_cast<double>(estimated_clock_offset_ns_) / 1e6,
        static_cast<double>(corrected_residual_ns) / 1e6,
        output_msg.point_num,
        invalid_time_points);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF错误: %s", ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "双雷达融合错误: %s", ex.what());
    }
  }

  message_filters::Subscriber<LivoxCustomMsg> sub1_;
  message_filters::Subscriber<LivoxCustomMsg> sub2_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
  rclcpp::Publisher<LivoxCustomMsg>::SharedPtr pub_merged_cloud_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Affine3d lidar2_to_target_ = Eigen::Affine3d::Identity();
  bool transform_initialized_ = false;

  std::string topic_1_;
  std::string topic_2_;
  std::string target_frame_id_;

  double sync_max_delay_ = 0.050;
  int sync_queue_size_ = 20;

  bool enable_software_rebase_ = true;
  double rebase_alpha_ = 0.02;
  double rebase_max_jump_sec_ = 0.010;
  int64_t rebase_max_jump_ns_ = 10000000LL;
  bool clock_offset_initialized_ = false;
  int64_t estimated_clock_offset_ns_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_output>());
  rclcpp::shutdown();
  return 0;
}