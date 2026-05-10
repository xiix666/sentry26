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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  // this->declare_parameter("map_frame", "map");
  // this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("map_frame", "odom_init");
  this->declare_parameter("odom_frame", "camera_init");
  // this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("min_accumulated_scans", 10);  // 累积至少多少帧点云
  this->declare_parameter("min_accumulated_points", 3000); // 累积至少多少个点
  this->declare_parameter("max_accumulation_time_sec", 3.0); // 最多累计时间

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("min_accumulated_scans", min_accumulated_scans_);
  this->get_parameter("min_accumulated_points", min_accumulated_points_);
  this->get_parameter("max_accumulation_time_sec", max_accumulation_time_sec_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  result_t_ = Eigen::Isometry3d::Identity();
  previous_result_t_ = Eigen::Isometry3d::Identity();

  loadGlobalMap(prior_pcd_file_);
  prepareTargetCloud();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    // "registered_scan", 10,
      "cloud_registered", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
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

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded target PCD file: %s, points = %zu",
    file_name.c_str(), global_map_->points.size());
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (initial_registration_done_) {
    return;
  }
  if (!accumulation_started_) {
    accumulation_started_ = true;
    accumulation_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Start accumulating initial source cloud.");
  }

  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  if (scan->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty scan, ignore it.");
    return;
  }

  *accumulated_cloud_ += *scan;
  accumulated_scan_count_++;

  if (accumulated_cloud_->size() > 100000) {
    RCLCPP_WARN(this->get_logger(), "Accumulated cloud too large, clearing old points.");
    accumulated_cloud_->clear();
  }
}
void SmallGicpRelocalizationNode::prepareTargetCloud()
{
  if (!global_map_ || global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Global target cloud is empty.");
    return;
  }

  target_ = small_gicp::voxelgrid_sampling_omp<  //下采样
    pcl::PointCloud<pcl::PointXYZ>,
    pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, global_leaf_size_);

  if (!target_ || target_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Target cloud after downsampling is empty.");
    return;
  }

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ =
    std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  RCLCPP_INFO(
    this->get_logger(),
    "Prepared target cloud. raw points = %zu, downsampled points = %zu",
    global_map_->size(), target_->size());
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (initial_registration_done_ || initial_registration_attempted_) {
    return;
  }

  if (!target_ || !target_tree_ || target_->empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Target cloud is not ready.");
    return;
  }

  if (!accumulation_started_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for initial source cloud.");
    return;
  }
  const double accumulated_time =
    (this->now() - accumulation_start_time_).seconds();

  const bool enough_scans =
    accumulated_scan_count_ >= min_accumulated_scans_;

  const bool enough_points =
    accumulated_cloud_->size() >= static_cast<size_t>(min_accumulated_points_);

  const bool timeout =
    accumulated_time >= max_accumulation_time_sec_;
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>,
    pcl::PointCloud<pcl::PointCovariance>>(
      *accumulated_cloud_, registered_leaf_size_);
  if (!(enough_scans && enough_points) && !timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Accumulating before initial GICP: scans = %d/%d, points = %zu/%d, time = %.2f/%.2f",
      accumulated_scan_count_,
      min_accumulated_scans_,
      accumulated_cloud_->size(),
      min_accumulated_points_,
      accumulated_time,
      max_accumulation_time_sec_);
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

  initial_registration_attempted_ = true;  //只尝试一次
  if (!source_ || source_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Source cloud after downsampling is empty.");
    return;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ =
    std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 30;

  RCLCPP_INFO(
    this->get_logger(),
    "Start initial GICP. target = %zu, source = %zu",
    target_->size(), source_->size());

  // result.T_target_source: source -> target
  // 这里就是 T_odom_init_camera_init
  auto result = register_->align(
    *target_,
    *source_,
    *target_tree_,
    previous_result_t_);

  if (result.converged) {
    result_t_ = result.T_target_source;
    previous_result_t_ = result_t_;
    initial_registration_done_ = true;

    const auto t = result_t_.translation();
    const Eigen::Quaterniond q(result_t_.rotation());

    RCLCPP_INFO(
      this->get_logger(),
      "Initial GICP converged. Publish %s -> %s. "
      "t = [%.3f, %.3f, %.3f], q = [%.6f, %.6f, %.6f, %.6f]",
      map_frame_.c_str(), odom_frame_.c_str(),
      t.x(), t.y(), t.z(),
      q.x(), q.y(), q.z(), q.w());

    accumulated_cloud_->clear();

    if (register_timer_) {
      register_timer_->cancel();
    }
  } else {
    initial_registration_done_ = false;

    RCLCPP_ERROR(
      this->get_logger(),
      "Initial GICP failed. No TF will be published. Stop relocalization.");

    accumulated_cloud_->clear();

    if (register_timer_) {
      register_timer_->cancel();
    }

    if (transform_timer_) {
      transform_timer_->cancel();
    }
  }
}
void SmallGicpRelocalizationNode::publishTransform()
{
  if (!initial_registration_done_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = map_frame_;   // odom_init
  transform_stamped.child_frame_id = odom_frame_;   // camera_init

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();

  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
