#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options)
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);

  this->declare_parameter("map_frame", "odom_init");
  this->declare_parameter("odom_frame", "camera_init");
  this->declare_parameter("prior_pcd_file", "");

  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->declare_parameter("min_accumulated_scans", 10);
  this->declare_parameter("min_accumulated_points", 3000);
  this->declare_parameter("max_accumulation_time_sec", 3.0);
  this->declare_parameter("max_relocalization_points", 100000);
  this->declare_parameter("max_relocalization_scans", 50);

  this->declare_parameter("fixed_bias_x", 0.0);
  this->declare_parameter("fixed_bias_y", 0.0);
  this->declare_parameter("fixed_bias_z", 0.0);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);

  this->get_parameter("min_accumulated_scans", min_accumulated_scans_);
  this->get_parameter("min_accumulated_points", min_accumulated_points_);
  this->get_parameter("max_accumulation_time_sec", max_accumulation_time_sec_);

  int max_relocalization_points_param = 100000;
  this->get_parameter("max_relocalization_points", max_relocalization_points_param);

  max_relocalization_points_ = static_cast<std::size_t>(std::max(0, max_relocalization_points_param));

  this->get_parameter("max_relocalization_scans", max_relocalization_scans_);

  double bias_x = 0.0;
  double bias_y = 0.0;
  double bias_z = 0.0;

  this->get_parameter("fixed_bias_x", bias_x);
  this->get_parameter("fixed_bias_y", bias_y);
  this->get_parameter("fixed_bias_z", bias_z);

  fixed_bias_ = Eigen::Isometry3d::Identity();
  fixed_bias_.translation() << bias_x, bias_y, bias_z;

  result_t_ = Eigen::Isometry3d::Identity();

  if (init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];

    result_t_.linear() = (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()))
                           .toRotationMatrix();
  }

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);
  prepareTargetCloud();

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_registered", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                            std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (file_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "prior_pcd_file is empty.");
    return;
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded target PCD file: %s, points = %zu", file_name.c_str(),
              global_map_->points.size());
}

void SmallGicpRelocalizationNode::registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (initial_registration_done_ || relocalization_failed_) {
    return;
  }

  if (!accumulation_started_) {
    accumulation_started_ = true;
    accumulation_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Start accumulating initial source cloud.");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msg, *scan);

  if (scan->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty scan, ignore it.");
    return;
  }

  *accumulated_cloud_ += *scan;
  ++accumulated_scan_count_;

  if (accumulated_cloud_->size() > max_relocalization_points_ || accumulated_scan_count_ > max_relocalization_scans_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Relocalization accumulation limit exceeded. "
                 "Cancel relocalization.");

    cancelRelocalization();
  }
}

void SmallGicpRelocalizationNode::cancelRelocalization()
{
  relocalization_failed_ = true;

  if (accumulated_cloud_) {
    accumulated_cloud_->clear();
  }

  if (register_timer_) {
    register_timer_->cancel();
  }

  if (transform_timer_) {
    transform_timer_->cancel();
  }

  RCLCPP_ERROR(this->get_logger(), "Relocalization canceled.");
}

void SmallGicpRelocalizationNode::prepareTargetCloud()
{
  if (!global_map_ || global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Global target cloud is empty.");
    return;
  }

  target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  if (!target_ || target_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Target cloud after downsampling is empty.");
    return;
  }

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  RCLCPP_INFO(this->get_logger(),
              "Prepared target cloud. raw points = %zu, "
              "downsampled points = %zu",
              global_map_->size(), target_->size());
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (initial_registration_done_ || initial_registration_attempted_) {
    return;
  }

  if (!target_ || !target_tree_ || target_->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Target cloud is not ready.");
    return;
  }

  if (!accumulation_started_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial source cloud.");
    return;
  }

  const double accumulated_time = (this->now() - accumulation_start_time_).seconds();

  const bool enough_scans = accumulated_scan_count_ >= min_accumulated_scans_;

  const bool enough_points = accumulated_cloud_->size() >= static_cast<std::size_t>(min_accumulated_points_);

  const bool timeout = accumulated_time >= max_accumulation_time_sec_;

  if (!(enough_scans && enough_points) && !timeout) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Accumulating before initial GICP: "
                         "scans = %d/%d, points = %zu/%d, "
                         "time = %.2f/%.2f",
                         accumulated_scan_count_, min_accumulated_scans_, accumulated_cloud_->size(),
                         min_accumulated_points_, accumulated_time, max_accumulation_time_sec_);
    return;
  }

  if (accumulated_cloud_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "No source points. Initial GICP aborted.");

    initial_registration_attempted_ = true;

    if (register_timer_) {
      register_timer_->cancel();
    }

    if (transform_timer_) {
      transform_timer_->cancel();
    }
    return;
  }

  initial_registration_attempted_ = true;

  source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  if (!source_ || source_->empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Source cloud after downsampling is empty. "
                 "Disable relocalization.");

    cancelRelocalization();
    return;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  register_->reduction.num_threads = num_threads_;

  register_->rejector.max_dist_sq = max_dist_sq_;

  register_->optimizer.max_iterations = 30;

  RCLCPP_INFO(this->get_logger(), "Start initial GICP. target = %zu, source = %zu", target_->size(), source_->size());

  auto result = register_->align(*target_, *source_, *target_tree_, result_t_);

  if (result.converged || result.error < max_registration_error_) {
    result_t_ = result.T_target_source;

    initial_registration_done_ = true;

    const auto t = result_t_.translation();

    const Eigen::Quaterniond q(result_t_.rotation());

    RCLCPP_INFO(this->get_logger(),
                "Initial GICP converged. Publish %s -> %s. "
                "t = [%.3f, %.3f, %.3f], "
                "q = [%.6f, %.6f, %.6f, %.6f]",
                map_frame_.c_str(), odom_frame_.c_str(), t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());

    accumulated_cloud_->clear();

    if (register_timer_) {
      register_timer_->cancel();
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Initial GICP failed. Disable relocalization.");

    cancelRelocalization();
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!initial_registration_done_) {
    return;
  }

  const Eigen::Isometry3d final_tf = fixed_bias_ * result_t_;

  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp = this->now();

  transform_stamped.header.frame_id = map_frame_;

  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = final_tf.translation();

  const Eigen::Quaterniond rotation(final_tf.rotation());

  transform_stamped.transform.translation.x = translation.x();

  transform_stamped.transform.translation.y = translation.y();

  transform_stamped.transform.translation.z = translation.z();

  transform_stamped.transform.rotation.x = rotation.x();

  transform_stamped.transform.rotation.y = rotation.y();

  transform_stamped.transform.rotation.z = rotation.z();

  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
