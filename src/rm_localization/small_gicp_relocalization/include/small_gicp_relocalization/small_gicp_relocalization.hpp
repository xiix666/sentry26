#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void prepareTargetCloud();
  void performRegistration();
  void publishTransform();
  void cancelRelocalization();

  int num_threads_{4};
  int num_neighbors_{20};
  float global_leaf_size_{0.25F};
  float registered_leaf_size_{0.25F};
  float max_dist_sq_{1.0F};
  double max_registration_error_{0.5};

  std::vector<double> init_pose_;

  bool initial_registration_done_{false};
  bool initial_registration_attempted_{false};
  bool relocalization_failed_{false};
  bool accumulation_started_{false};

  int accumulated_scan_count_{0};
  int min_accumulated_scans_{10};
  int min_accumulated_points_{3000};
  std::size_t max_relocalization_points_{100000};
  int max_relocalization_scans_{50};
  double max_accumulation_time_sec_{3.0};
  rclcpp::Time accumulation_start_time_;

  std::string map_frame_;
  std::string odom_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string prior_pcd_file_;

  Eigen::Isometry3d result_t_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d fixed_bias_{Eigen::Isometry3d::Identity()};

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;

  std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}

#endif
