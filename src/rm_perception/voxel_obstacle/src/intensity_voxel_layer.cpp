#include "intensity_voxel_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <stdexcept>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

#define VOXEL_BITS 16

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::Observation;

namespace xx_nav2_costmap_2d {
void IntensityVoxelLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error(
        "IntensityVoxelLayer: lifecycle node expired during initialization");
  }

  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();

  node->get_parameter(name_ + ".footprint_clearing_enabled",
                      footprint_clearing_enabled_);
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".combination_method", combination_method_);

  node->declare_parameter(name_ + ".obstacle_hold_time", 1.5);
  node->get_parameter(name_ + ".obstacle_hold_time", obstacle_hold_time_);

  node->declare_parameter(name_ + ".z_voxels", 16);
  node->get_parameter(name_ + ".z_voxels", size_z_);

  node->declare_parameter(name_ + ".origin_z", 16.0);
  node->get_parameter(name_ + ".origin_z", origin_z_);

  node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  node->get_parameter(name_ + ".min_obstacle_intensity",
                      min_obstacle_intensity_);

  node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  node->get_parameter(name_ + ".max_obstacle_intensity",
                      max_obstacle_intensity_);

  node->declare_parameter(name_ + ".z_resolution", 0.05);
  node->get_parameter(name_ + ".z_resolution", z_resolution_);

  node->declare_parameter(name_ + ".mark_threshold", 0);
  node->get_parameter(name_ + ".mark_threshold", mark_threshold_);

  node->declare_parameter(name_ + ".publish_voxel_map", false);
  node->get_parameter(name_ + ".publish_voxel_map", publish_voxel_);

  node->declare_parameter(name_ + ".max_gradient_threshold", 1.0);
  node->get_parameter(name_ + ".max_gradient_threshold",
                      max_gradient_threshold_);

  node->declare_parameter(name_ + ".low_gradient_threshold", 0.2);
  node->get_parameter(name_ + ".low_gradient_threshold",
                      low_gradient_threshold_);

  node->declare_parameter(name_ + ".obstacle_expand_size", 1);
  node->get_parameter(name_ + ".obstacle_expand_size", obstacle_expand_size_);

  node->declare_parameter(name_ + ".near_obstacle_radius", 0.6);
  node->get_parameter(name_ + ".near_obstacle_radius", near_obstacle_radius_);

  node->declare_parameter(name_ + ".point_cluster_tolerance", 0.15);
  node->get_parameter(name_ + ".point_cluster_tolerance",
                      point_cluster_tolerance_);

  node->declare_parameter(name_ + ".min_cluster_points", 8);
  node->get_parameter(name_ + ".min_cluster_points", min_cluster_points_);

  node->declare_parameter(name_ + ".cluster_hard_evidence_min_points", 6);
  node->get_parameter(name_ + ".cluster_hard_evidence_min_points",
                      cluster_hard_evidence_min_points_);

  node->declare_parameter(name_ + ".soft_min_cluster_points", 3);
  node->get_parameter(name_ + ".soft_min_cluster_points",
                      soft_min_cluster_points_);

  node->declare_parameter(name_ + ".soft_score_threshold", 0.55);
  node->get_parameter(name_ + ".soft_score_threshold", soft_score_threshold_);

  node->declare_parameter(name_ + ".speed_low_confidence_mps", 4.0);
  node->get_parameter(name_ + ".speed_low_confidence_mps",
                      speed_low_confidence_mps_);

  node->declare_parameter(name_ + ".use_static_obstacle_area", false);
  node->get_parameter(name_ + ".use_static_obstacle_area",
                      use_static_obstacle_area_);

  node->declare_parameter(name_ + ".static_obs_min_x", 0.0);
  node->declare_parameter(name_ + ".static_obs_min_y", 0.0);
  node->declare_parameter(name_ + ".static_obs_max_x", 0.0);
  node->declare_parameter(name_ + ".static_obs_max_y", 0.0);
  node->declare_parameter(name_ + ".static_obs2_min_x", 0.0);
  node->declare_parameter(name_ + ".static_obs2_min_y", 0.0);
  node->declare_parameter(name_ + ".static_obs2_max_x", 0.0);
  node->declare_parameter(name_ + ".static_obs2_max_y", 0.0);

  node->get_parameter(name_ + ".static_obs_min_x", static_obs_min_x_);
  node->get_parameter(name_ + ".static_obs_min_y", static_obs_min_y_);
  node->get_parameter(name_ + ".static_obs_max_x", static_obs_max_x_);
  node->get_parameter(name_ + ".static_obs_max_y", static_obs_max_y_);
  node->get_parameter(name_ + ".static_obs2_min_x", static_obs2_min_x_);
  node->get_parameter(name_ + ".static_obs2_min_y", static_obs2_min_y_);
  node->get_parameter(name_ + ".static_obs2_max_x", static_obs2_max_x_);
  node->get_parameter(name_ + ".static_obs2_max_y", static_obs2_max_y_);

  node->declare_parameter(name_ + ".disable_dynamic_map_on_tilt", true);
  node->declare_parameter(name_ + ".tilt_frame", std::string("base_link"));
  node->declare_parameter(name_ + ".tilt_disable_threshold_deg", 20.0);
  node->declare_parameter(name_ + ".tilt_recover_threshold_deg", 15.0);
  node->declare_parameter(name_ + ".publish_pass_enable", false);

  node->get_parameter(name_ + ".publish_pass_enable", publish_pass_enable_);
  node->get_parameter(name_ + ".disable_dynamic_map_on_tilt",
                      disable_dynamic_map_on_tilt_);
  node->get_parameter(name_ + ".tilt_frame", tilt_frame_);
  node->get_parameter(name_ + ".tilt_disable_threshold_deg",
                      tilt_disable_threshold_deg_);
  node->get_parameter(name_ + ".tilt_recover_threshold_deg",
                      tilt_recover_threshold_deg_);

  node->declare_parameter(name_ + ".velocity_odom_topic",
                          std::string("/aft_mapped_to_init"));
  node->get_parameter(name_ + ".velocity_odom_topic", velocity_odom_topic_);
  node->declare_parameter(name_ + ".height_score_full", 0.35);

  node->get_parameter(name_ + ".height_score_full", height_score_full_);

  node->declare_parameter(name_ + ".acceleration_suppression_threshold_mps2",
                          3.0);

  node->get_parameter(name_ + ".acceleration_suppression_threshold_mps2",
                      acceleration_suppression_threshold_mps2_);

  node->declare_parameter(name_ + ".acceleration_suppression_radius", 2.0);

  node->get_parameter(name_ + ".acceleration_suppression_radius",
                      acceleration_suppression_radius_);
  node->declare_parameter(name_ + ".acceleration_suppression_hold_time", 0.8);

  node->get_parameter(name_ + ".acceleration_suppression_hold_time",
                      acceleration_suppression_hold_time_);

  node->declare_parameter(name_ + ".enable_sudden_obstacle_history_filter",
                          true);
  node->get_parameter(name_ + ".enable_sudden_obstacle_history_filter",
                      enable_sudden_obstacle_history_filter_);

  node->declare_parameter(name_ + ".sudden_obstacle_near_radius", 2.5);
  node->get_parameter(name_ + ".sudden_obstacle_near_radius",
                      sudden_obstacle_near_radius_);

  node->declare_parameter(name_ + ".sudden_obstacle_stable_free_duration", 1.0);
  node->get_parameter(name_ + ".sudden_obstacle_stable_free_duration",
                      sudden_obstacle_stable_free_duration_);

  node->declare_parameter(name_ + ".sudden_obstacle_max_observation_gap", 0.2);
  node->get_parameter(name_ + ".sudden_obstacle_max_observation_gap",
                      sudden_obstacle_max_observation_gap_);

  node->declare_parameter(name_ + ".sudden_obstacle_rising_cell_threshold", 30);
  node->get_parameter(name_ + ".sudden_obstacle_rising_cell_threshold",
                      sudden_obstacle_rising_cell_threshold_);

  node->declare_parameter(name_ + ".sudden_obstacle_history_retention", 5.0);
  node->get_parameter(name_ + ".sudden_obstacle_history_retention",
                      sudden_obstacle_history_retention_);

  size_z_ = std::clamp(size_z_, 1, VOXEL_BITS);
  z_resolution_ = std::max(1e-3, z_resolution_);
  obstacle_hold_time_ = std::max(0.0, obstacle_hold_time_);
  obstacle_expand_size_ = std::max(0, obstacle_expand_size_);
  point_cluster_tolerance_ = std::max(0.01, point_cluster_tolerance_);
  soft_min_cluster_points_ = std::max(1, soft_min_cluster_points_);
  min_cluster_points_ = std::max(soft_min_cluster_points_, min_cluster_points_);
  cluster_hard_evidence_min_points_ =
      std::clamp(cluster_hard_evidence_min_points_, 1, min_cluster_points_);
  soft_score_threshold_ = clamp01(soft_score_threshold_);
  near_obstacle_radius_ = std::max(0.05, near_obstacle_radius_);
  speed_low_confidence_mps_ = std::max(0.1, speed_low_confidence_mps_);
  height_score_full_ =
      std::max(min_obstacle_intensity_ + 0.01, height_score_full_);
  acceleration_suppression_threshold_mps2_ =
      std::max(0.1, acceleration_suppression_threshold_mps2_);

  acceleration_suppression_radius_ =
      std::max(0.0, acceleration_suppression_radius_);
  acceleration_suppression_hold_time_ =
      std::max(0.0, acceleration_suppression_hold_time_);
  sudden_obstacle_near_radius_ = std::max(0.1, sudden_obstacle_near_radius_);

  sudden_obstacle_stable_free_duration_ =
      std::max(0.1, sudden_obstacle_stable_free_duration_);

  sudden_obstacle_max_observation_gap_ =
      std::max(0.05, sudden_obstacle_max_observation_gap_);

  sudden_obstacle_rising_cell_threshold_ =
      std::max(1, sudden_obstacle_rising_cell_threshold_);

  sudden_obstacle_history_retention_ =
      std::max(sudden_obstacle_stable_free_duration_ + 1.0,
               sudden_obstacle_history_retention_);

  if (max_gradient_threshold_ <= low_gradient_threshold_) {
    RCLCPP_WARN(logger_,
                "max_gradient_threshold(%.3f) must be greater than "
                "low_gradient_threshold(%.3f). "
                "Force max=low+0.001.",
                max_gradient_threshold_, low_gradient_threshold_);
    max_gradient_threshold_ = low_gradient_threshold_ + 0.001;
  }

  if (publish_voxel_) {
    voxel_pub_ =
        node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  rm_task_sub_ = node->create_subscription<std_msgs::msg::Int32>(
      "/rm_task", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        rm_task_.store(msg->data, std::memory_order_relaxed);
      });
  if (publish_pass_enable_) {
    area_obstacle_flag_pub_ = node->create_publisher<std_msgs::msg::Int32>(
        "/pass_enable", rclcpp::QoS(10));
  }

  lio_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      velocity_odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto &velocity = msg->twist.twist.linear;
        double speed =
            std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                      velocity.z * velocity.z);
        if (!std::isfinite(speed)) {
          return;
        }

        if (speed < 0.08) {
          speed = 0.0;
        }

        current_speed_mps_.store(speed, std::memory_order_relaxed);
        double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

        if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
          stamp_sec = clock_->now().seconds();
        }

        double filtered_acceleration = 0.0;

        if (has_last_odom_speed_) {
          const double dt = stamp_sec - last_odom_stamp_sec_;

          if (dt >= 0.005 && dt <= 0.5) {
            const double raw_acceleration =
                std::abs(speed - last_odom_speed_mps_) / dt;

            if (std::isfinite(raw_acceleration)) {
              const double previous_acceleration =
                  current_acceleration_mps2_.load(std::memory_order_relaxed);

              filtered_acceleration =
                  0.60 * previous_acceleration + 0.40 * raw_acceleration;
            }
          }
        }

        current_acceleration_mps2_.store(filtered_acceleration,
                                         std::memory_order_relaxed);

        acceleration_update_time_sec_.store(clock_->now().seconds(),
                                            std::memory_order_relaxed);

        last_odom_speed_mps_ = speed;

        last_odom_stamp_sec_ = stamp_sec;

        has_last_odom_speed_ = true;
      });

  matchSize();
}

IntensityVoxelLayer::~IntensityVoxelLayer() = default;

void IntensityVoxelLayer::updateFootprint(double robot_x, double robot_y,
                                          double robot_yaw, double *min_x,
                                          double *min_y, double *max_x,
                                          double *max_y) {
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw,
                                      getFootprint(), transformed_footprint_);

  for (auto &point : transformed_footprint_) {
    touch(point.x, point.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize() {
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);

  active_obstacle_cells_.clear();
  resetSuddenObstacleHistory();
}

void IntensityVoxelLayer::markStaticObstacleArea(double robot_x, double robot_y,
                                                 double *min_x, double *min_y,
                                                 double *max_x, double *max_y) {
  if (!use_static_obstacle_area_) {
    return;
  }

  const double x1 = 8.784;
  const double y1 = 5.488;
  const double x2 = 5.861;
  const double y2 = 1.669;
  const double px_limit = 6.021;
  const double py_limit = -5.504;
  const double vx = x1 - x2;
  const double vy = y1 - y2;
  const double qx = robot_x - x2;
  const double qy = robot_y - y2;
  const double cross = vx * qy - vy * qx;
  const bool left_down_of_line = cross >= 0.0;
  const bool left_down_of_point = robot_x <= px_limit && robot_y >= py_limit;
  const bool robot_in_trigger_region = left_down_of_line || left_down_of_point;
  const double block_px = 13.752;
  const double block_py = 4.439;
  const bool right_up_of_block_point =
      robot_x >= block_px && robot_y <= block_py;
  const double rx1 = 15.110;
  const double ry1 = -2.326;
  const double rx2 = 12.343;
  const double ry2 = -6.385;
  const double rvx = rx1 - rx2;
  const double rvy = ry1 - ry2;
  const double rqx = robot_x - rx2;
  const double rqy = robot_y - ry2;
  const double r_cross = rvx * rqy - rvy * rqx;
  const bool right_up_of_block_line = r_cross <= 0.0;
  const bool robot_in_block_region =
      right_up_of_block_point || right_up_of_block_line;

  if (!robot_in_trigger_region && !robot_in_block_region) {
    return;
  }

  auto touch_rect_area = [&](double rect_min_x, double rect_min_y,
                             double rect_max_x, double rect_max_y) {
    const double min_wx = std::min(rect_min_x, rect_max_x);
    const double max_wx = std::max(rect_min_x, rect_max_x);
    const double min_wy = std::min(rect_min_y, rect_max_y);
    const double max_wy = std::max(rect_min_y, rect_max_y);

    touch(min_wx, min_wy, min_x, min_y, max_x, max_y);
    touch(max_wx, max_wy, min_x, min_y, max_x, max_y);
  };

  if (rm_task_.load(std::memory_order_relaxed) == 2) {
    touch_rect_area(static_obs_min_x_, static_obs_min_y_,
                    static_obs_max_x_, static_obs_max_y_);
    touch_rect_area(static_obs2_min_x_, static_obs2_min_y_,
                    static_obs2_max_x_, static_obs2_max_y_);
    return;
  }

  auto mark_rect_obstacle = [&](double rect_min_x, double rect_min_y,
                                double rect_max_x, double rect_max_y) {
    const double min_wx = std::min(rect_min_x, rect_max_x);
    const double max_wx = std::max(rect_min_x, rect_max_x);
    const double min_wy = std::min(rect_min_y, rect_max_y);
    const double max_wy = std::max(rect_min_y, rect_max_y);
    unsigned int min_mx = 0;
    unsigned int min_my = 0;
    unsigned int max_mx = 0;
    unsigned int max_my = 0;

    if (!worldToMap(min_wx, min_wy, min_mx, min_my)) {
      return;
    }
    if (!worldToMap(max_wx, max_wy, max_mx, max_my)) {
      return;
    }

    const unsigned int start_x = std::min(min_mx, max_mx);
    const unsigned int end_x = std::max(min_mx, max_mx);
    const unsigned int start_y = std::min(min_my, max_my);
    const unsigned int end_y = std::max(min_my, max_my);

    for (unsigned int mx = start_x; mx <= end_x; ++mx) {
      for (unsigned int my = start_y; my <= end_y; ++my) {
        costmap_[getIndex(mx, my)] = LETHAL_OBSTACLE;
      }
    }

    touch(min_wx, min_wy, min_x, min_y, max_x, max_y);
    touch(max_wx, max_wy, min_x, min_y, max_x, max_y);
  };

  mark_rect_obstacle(static_obs_min_x_, static_obs_min_y_, static_obs_max_x_,
                     static_obs_max_y_);
  mark_rect_obstacle(static_obs2_min_x_, static_obs2_min_y_, static_obs2_max_x_,
                     static_obs2_max_y_);
}

void IntensityVoxelLayer::markObstacleWithExpand(unsigned int mx,
                                                 unsigned int my, double wx,
                                                 double wy, double *min_x,
                                                 double *min_y, double *max_x,
                                                 double *max_y) {
  for (int dx = -obstacle_expand_size_; dx <= obstacle_expand_size_; ++dx) {
    for (int dy = -obstacle_expand_size_; dy <= obstacle_expand_size_; ++dy) {
      const int cell_x = static_cast<int>(mx) + dx;
      const int cell_y = static_cast<int>(my) + dy;

      if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(size_x_) ||
          cell_y >= static_cast<int>(size_y_)) {
        continue;
      }

      costmap_[getIndex(static_cast<unsigned int>(cell_x),
                        static_cast<unsigned int>(cell_y))] = LETHAL_OBSTACLE;
    }
  }

  const double expand_distance = obstacle_expand_size_ * resolution_;
  touch(wx - expand_distance, wy - expand_distance, min_x, min_y, max_x, max_y);
  touch(wx + expand_distance, wy + expand_distance, min_x, min_y, max_x, max_y);
}

void IntensityVoxelLayer::reset() {
  ObstacleLayer::reset();
  resetMaps();

  active_obstacle_cells_.clear();
  resetSuddenObstacleHistory();
  current_speed_mps_.store(0.0, std::memory_order_relaxed);

  current_acceleration_mps2_.store(0.0, std::memory_order_relaxed);

  acceleration_update_time_sec_.store(-1.0, std::memory_order_relaxed);
  acceleration_suppression_until_sec_.store(-1.0, std::memory_order_relaxed);
  has_last_odom_speed_ = false;
  last_odom_speed_mps_ = 0.0;
  last_odom_stamp_sec_ = -1.0;
}

void IntensityVoxelLayer::resetMaps() {
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
}

void IntensityVoxelLayer::updateBounds(double robot_x, double robot_y,
                                       double robot_yaw, double *min_x,
                                       double *min_y, double *max_x,
                                       double *max_y) {
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0,
                 robot_y - getSizeInMetersY() / 2.0);
  }

  resetMaps();

  if (!enabled_) {
    publishAreaObstacleFlag(robot_x, true);

    return;
  }

  const bool was_suppressed = dynamic_map_suppressed_by_tilt_;
  const bool suppress_dynamic_map = updateTiltSuppressionState();

  if (suppress_dynamic_map && !was_suppressed) {
    clearDynamicObstacleHistory(min_x, min_y, max_x, max_y);
    resetSuddenObstacleHistory();
  }

  markStaticObstacleArea(robot_x, robot_y, min_x, min_y, max_x, max_y);
  useExtraBounds(min_x, min_y, max_x, max_y);
  const int rm_task = rm_task_.load(std::memory_order_relaxed);
  const bool robot_in_dynamic_clear_region =
      isInDynamicClearTriggerArea(robot_x, robot_y);
  const bool robot_in_forced_area_clear_region =
      isInForcedAreaClearTriggerRegion(robot_x, robot_y);
  const bool clear_forced_area =
      rm_task == 1 || robot_in_forced_area_clear_region;

  if (clear_forced_area) {
    clearDynamicObstacleHistoryInForcedAreas(min_x, min_y, max_x, max_y);

    updateForcedDynamicObstacleAreas(false, min_x, min_y, max_x, max_y);
  } else {
    updateForcedDynamicObstacleAreas(true, min_x, min_y, max_x, max_y);
  }

  const bool clear_other_dynamic_obstacles =
      (rm_task == 2 && task_clear) || robot_in_dynamic_clear_region;

  if (rm_task != 1 && clear_other_dynamic_obstacles) {
    clearDynamicObstacleHistoryOutsideForcedAreas(min_x, min_y, max_x, max_y);

    resetSuddenObstacleHistory();

    current_ = true;

    publishAreaObstacleFlag(robot_x, false);

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

    return;
  }

  auto publish_voxel_grid = [&]() {
    if (!publish_voxel_ || !voxel_pub_) {
      return;
    }

    nav2_msgs::msg::VoxelGrid grid_msg;
    const unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();

    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);

    if (size > 0) {
      std::memcpy(grid_msg.data.data(), voxel_grid_.getData(),
                  size * sizeof(unsigned int));
    }

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;
    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();

    voxel_pub_->publish(grid_msg);
  };

  if (suppress_dynamic_map) {
    current_ = true;
    publishAreaObstacleFlag(robot_x, true);
    publish_voxel_grid();
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    return;
  }

  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;
  current_ = current;
  const double now_sec = clock_->now().seconds();
  const double current_acceleration =
      current_acceleration_mps2_.load(std::memory_order_relaxed);
  const double acceleration_update_time =
      acceleration_update_time_sec_.load(std::memory_order_relaxed);
  const bool acceleration_is_fresh = acceleration_update_time > 0.0 &&
                                     now_sec - acceleration_update_time <= 0.5;
  const bool acceleration_triggered =
      acceleration_is_fresh &&
      current_acceleration >= acceleration_suppression_threshold_mps2_;

  if (acceleration_triggered) {
    const double new_suppression_until =
        now_sec + acceleration_suppression_hold_time_;
    const double old_suppression_until =
        acceleration_suppression_until_sec_.load(std::memory_order_relaxed);

    acceleration_suppression_until_sec_.store(
        std::max(old_suppression_until, new_suppression_until),
        std::memory_order_relaxed);
  }

  const double suppression_until =
      acceleration_suppression_until_sec_.load(std::memory_order_relaxed);
  const bool suppress_near_by_acceleration = now_sec <= suppression_until;
  const double acceleration_suppression_radius_sq =
      acceleration_suppression_radius_ * acceleration_suppression_radius_;
  if (suppress_near_by_acceleration) {
    clearNearDynamicObstacleHistory(robot_x, robot_y,
                                    acceleration_suppression_radius_, min_x,
                                    min_y, max_x, max_y);

    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 500,
        "Acceleration %.2f m/s^2 exceeds %.2f m/s^2. "
        "Immediately clear and suppress dynamic perception within %.2f m.",
        current_acceleration, acceleration_suppression_threshold_mps2_,
        acceleration_suppression_radius_);
  }

  std::vector<CandidateObstaclePoint> candidate_points;
  candidate_points.reserve(4096);

  for (const auto &observation : observations) {
    const double sq_max_range =
        observation.obstacle_max_range_ * observation.obstacle_max_range_;
    const double sq_min_range =
        observation.obstacle_min_range_ * observation.obstacle_min_range_;
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*observation.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*observation.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*observation.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*observation.cloud_,
                                                      "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> it_gradient(
        *observation.cloud_, "curvature");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_gradient) {
      const double px = *it_x;
      const double py = *it_y;
      const double pz = *it_z;
      const double relative_height = *it_i;
      const double gradient = *it_gradient;

      if (clear_forced_area && isInForcedDynamicObstacleArea(px, py)) {
        continue;
      }

      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
          !std::isfinite(relative_height) || !std::isfinite(gradient)) {
        continue;
      }

      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
        continue;
      }

      if (relative_height < min_obstacle_intensity_ ||
          relative_height > max_obstacle_intensity_) {
        continue;
      }

      if (gradient < 0.0) {
        continue;
      }

      const bool hard_gradient = gradient >= max_gradient_threshold_;
      const bool soft_mid_gradient = gradient >= low_gradient_threshold_ &&
                                     gradient < max_gradient_threshold_;

      if (!hard_gradient && !soft_mid_gradient) {
        continue;
      }

      const double diff_x = px - observation.origin_.x;
      const double diff_y = py - observation.origin_.y;
      const double diff_z = pz - observation.origin_.z;
      const double sq_range =
          diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

      if (sq_range <= sq_min_range || sq_range >= sq_max_range) {
        continue;
      }
      if (suppress_near_by_acceleration &&
          sq_range <= acceleration_suppression_radius_sq) {
        continue;
      }

      unsigned int mx = 0;
      unsigned int my = 0;
      unsigned int mz = 0;
      const double voxel_z = pz < origin_z_ ? origin_z_ : pz;
      if (!worldToMap3D(px, py, voxel_z, mx, my, mz)) {
        continue;
      }

      CandidateObstaclePoint candidate;
      candidate.x = px;
      candidate.y = py;
      candidate.z = pz;
      candidate.gradient = gradient;
      candidate.relative_height = relative_height;
      candidate.range = std::sqrt(sq_range);
      candidate.mx = mx;
      candidate.my = my;
      candidate.mz = mz;
      candidate.column_idx = getIndex(mx, my);
      candidate.hard_gradient = hard_gradient;
      candidate.soft_mid_gradient = soft_mid_gradient;

      candidate_points.push_back(candidate);
    }
  }

  constexpr double kAdaptiveNearRange = 5.0;
  constexpr double kAdaptiveFarRange = 10.0;
  constexpr double kFarHoldTimeBonus = 0.4;

  auto distance_adaptation_ratio = [&](double range) {
    if (!std::isfinite(range)) {
      return 0.0;
    }

    return clamp01((range - kAdaptiveNearRange) /
                   std::max(1e-6, kAdaptiveFarRange - kAdaptiveNearRange));
  };
  auto interpolate_required_points = [&](int near_required, double range) {
    const double ratio = distance_adaptation_ratio(range);
    const double interpolated =
        static_cast<double>(near_required) +
        ratio * (1.0 - static_cast<double>(near_required));

    return std::max(1, static_cast<int>(std::lround(interpolated)));
  };
  auto obstacle_hold_time_for_range = [&](double range) {
    return obstacle_hold_time_ +
           kFarHoldTimeBonus * distance_adaptation_ratio(range);
  };

  std::vector<std::vector<std::size_t>> point_clusters;
  clusterCandidatePoints(candidate_points, point_clusters, 1);

  std::unordered_set<unsigned int> accepted_columns;
  accepted_columns.reserve(candidate_points.size());

  const double tilt_confidence = clamp01(
      1.0 - current_tilt_deg_ / std::max(1.0, tilt_disable_threshold_deg_));
  const double speed = current_speed_mps_.load(std::memory_order_relaxed);
  const double speed_confidence =
      clamp01(1.0 - speed / std::max(0.1, speed_low_confidence_mps_));
  const double state_confidence =
      0.60 * tilt_confidence + 0.40 * speed_confidence;
  const double effective_soft_threshold =
      clamp01(soft_score_threshold_ +
              (0.9 - soft_score_threshold_) * (1.0 - state_confidence));

  for (const auto &cluster : point_clusters) {
    int hard_gradient_count = 0;
    int soft_point_count = 0;
    double soft_gradient_sum = 0.0;
    double soft_range_sum = 0.0;
    double soft_height_sum = 0.0;
    double cluster_range_sum = 0.0;

    std::unordered_set<unsigned int> cluster_columns;
    cluster_columns.reserve(cluster.size());

    for (const std::size_t point_index : cluster) {
      const auto &point = candidate_points[point_index];

      cluster_columns.insert(point.column_idx);
      cluster_range_sum += point.range;

      if (point.hard_gradient) {
        ++hard_gradient_count;
      }

      if (point.soft_mid_gradient) {
        ++soft_point_count;
        soft_gradient_sum += point.gradient;
        soft_range_sum += point.range;
        soft_height_sum += point.relative_height;
      }
    }

    if (cluster.empty()) {
      continue;
    }

    const double avg_cluster_range =
        cluster_range_sum / static_cast<double>(cluster.size());
    const int required_cluster_points =
        interpolate_required_points(min_cluster_points_, avg_cluster_range);
    const int required_hard_evidence_points = interpolate_required_points(
        cluster_hard_evidence_min_points_, avg_cluster_range);
    const int required_soft_points = interpolate_required_points(
        soft_min_cluster_points_, avg_cluster_range);
    const bool hard_cluster_pass =
        static_cast<int>(cluster.size()) >= required_cluster_points &&
        hard_gradient_count >= required_hard_evidence_points;

    if (hard_cluster_pass) {
      accepted_columns.insert(cluster_columns.begin(), cluster_columns.end());

      continue;
    }

    if (soft_point_count < required_soft_points) {
      continue;
    }

    const double point_count = static_cast<double>(soft_point_count);
    const double avg_gradient = soft_gradient_sum / point_count;
    const double avg_relative_height = soft_height_sum / point_count;
    const double avg_range = soft_range_sum / point_count;
    const double gradient_score = clamp01(
        (avg_gradient - low_gradient_threshold_) /
        std::max(1e-6, max_gradient_threshold_ - low_gradient_threshold_));
    const double height_score =
        clamp01((avg_relative_height - min_obstacle_intensity_) /
                std::max(1e-6, height_score_full_ - min_obstacle_intensity_));
    const double point_score =
        clamp01(static_cast<double>(soft_point_count) /
                static_cast<double>(std::max(1, required_cluster_points)));
    const double near_score =
        clamp01(1.0 - avg_range / std::max(0.05, near_obstacle_radius_));
    const double cluster_score = 0.4 * gradient_score + 0.3 * height_score +
                                 0.25 * point_score + 0.05 * near_score;

    if (cluster_score >= effective_soft_threshold) {
      accepted_columns.insert(cluster_columns.begin(), cluster_columns.end());
    }
  }

  if (enable_sudden_obstacle_history_filter_) {
    pruneObstacleCellTemporalStates(now_sec);
    const int robot_world_grid_x =
        static_cast<int>(std::floor(robot_x / resolution_));
    const int robot_world_grid_y =
        static_cast<int>(std::floor(robot_y / resolution_));
    const int near_radius_cells =
        static_cast<int>(std::ceil(sudden_obstacle_near_radius_ / resolution_));
    const double near_radius_sq =
        sudden_obstacle_near_radius_ * sudden_obstacle_near_radius_;
    std::unordered_set<std::uint64_t> current_near_obstacle_keys;

    current_near_obstacle_keys.reserve(accepted_columns.size() * 2);

    for (const unsigned int column_idx : accepted_columns) {
      if (column_idx >= size_x_ * size_y_) {
        continue;
      }

      const unsigned int mx = column_idx % size_x_;
      const unsigned int my = column_idx / size_x_;
      double wx = 0.0;
      double wy = 0.0;

      mapToWorld(mx, my, wx, wy);
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;

      if (dx * dx + dy * dy > near_radius_sq) {
        continue;
      }

      const int world_grid_x = static_cast<int>(std::floor(wx / resolution_));
      const int world_grid_y = static_cast<int>(std::floor(wy / resolution_));

      current_near_obstacle_keys.insert(
          makeObstacleHistoryKey(world_grid_x, world_grid_y));
    }

    std::size_t rising_obstacle_cell_count = 0;

    for (const std::uint64_t key : current_near_obstacle_keys) {
      const auto state_it = obstacle_cell_temporal_states_.find(key);

      if (state_it == obstacle_cell_temporal_states_.end()) {
        continue;
      }

      const auto &state = state_it->second;
      const bool observation_is_continuous =
          state.last_observed_sec > 0.0 &&
          now_sec - state.last_observed_sec <=
              sudden_obstacle_max_observation_gap_;
      const bool was_stably_free = !state.occupied &&
                                   state.free_since_sec > 0.0 &&
                                   now_sec - state.free_since_sec >=
                                       sudden_obstacle_stable_free_duration_;

      if (observation_is_continuous && was_stably_free) {
        ++rising_obstacle_cell_count;
      }
    }

    const bool drop_current_obstacle_frame =
        rising_obstacle_cell_count >=
        static_cast<std::size_t>(sudden_obstacle_rising_cell_threshold_);

    if (drop_current_obstacle_frame) {
      accepted_columns.clear();

      RCLCPP_WARN(logger_,
                  "Drop current obstacle frame: "
                  "%zu nearby world cells changed from stable-free "
                  "to occupied in one frame, threshold=%d, radius=%.2f m.",
                  rising_obstacle_cell_count,
                  sudden_obstacle_rising_cell_threshold_,
                  sudden_obstacle_near_radius_);
    }

    for (int offset_x = -near_radius_cells; offset_x <= near_radius_cells;
         ++offset_x) {
      for (int offset_y = -near_radius_cells; offset_y <= near_radius_cells;
           ++offset_y) {
        const int world_grid_x = robot_world_grid_x + offset_x;
        const int world_grid_y = robot_world_grid_y + offset_y;
        const double cell_center_x =
            (static_cast<double>(world_grid_x) + 0.5) * resolution_;
        const double cell_center_y =
            (static_cast<double>(world_grid_y) + 0.5) * resolution_;
        const double dx = cell_center_x - robot_x;
        const double dy = cell_center_y - robot_y;

        if (dx * dx + dy * dy > near_radius_sq) {
          continue;
        }

        const std::uint64_t key =
            makeObstacleHistoryKey(world_grid_x, world_grid_y);
        auto &state = obstacle_cell_temporal_states_[key];
        const bool occupied_this_frame = !drop_current_obstacle_frame &&
                                         current_near_obstacle_keys.find(key) !=
                                             current_near_obstacle_keys.end();

        if (occupied_this_frame) {
          state.occupied = true;
          state.free_since_sec = -1.0;
        } else {
          if (state.occupied || state.free_since_sec < 0.0) {
            state.free_since_sec = now_sec;
          }

          state.occupied = false;
        }

        state.last_observed_sec = now_sec;
      }
    }
  }

  for (const unsigned int column_idx : accepted_columns) {
    if (column_idx >= size_x_ * size_y_) {
      continue;
    }

    const unsigned int mx = column_idx % size_x_;
    const unsigned int my = column_idx / size_x_;
    double wx = 0.0;
    double wy = 0.0;
    mapToWorld(mx, my, wx, wy);
    if (suppress_near_by_acceleration) {
      const double expand_distance = obstacle_expand_size_ * resolution_;
      const double effective_clear_radius = acceleration_suppression_radius_ +
                                            std::sqrt(2.0) * expand_distance +
                                            0.5 * resolution_;
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;

      if (dx * dx + dy * dy <=
          effective_clear_radius * effective_clear_radius) {
        continue;
      }
    }
    bool found = false;
    for (auto &cell : active_obstacle_cells_) {
      if (std::hypot(cell.wx - wx, cell.wy - wy) < resolution_ * 0.5) {
        cell.last_hit_time = now_sec;
        found = true;
        break;
      }
    }

    if (!found) {
      active_obstacle_cells_.push_back(ActiveObstacleCell{wx, wy, now_sec});
    }
  }

  for (auto it = active_obstacle_cells_.begin();
       it != active_obstacle_cells_.end();) {
    const double obstacle_range =
        std::hypot(it->wx - robot_x, it->wy - robot_y);
    const double effective_hold_time =
        obstacle_hold_time_for_range(obstacle_range);
    const bool expired = now_sec - it->last_hit_time > effective_hold_time;

    if (expired) {
      const double expand_distance = obstacle_expand_size_ * resolution_;
      touch(it->wx - expand_distance, it->wy - expand_distance,
            min_x, min_y, max_x, max_y);
      touch(it->wx + expand_distance, it->wy + expand_distance,
            min_x, min_y, max_x, max_y);

      it = active_obstacle_cells_.erase(it);
      continue;
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (worldToMap(it->wx, it->wy, mx, my)) {
      markObstacleWithExpand(mx, my, it->wx, it->wy,
                             min_x, min_y, max_x, max_y);
    }

    ++it;
  }

  for (const auto &point : candidate_points) {
    if (accepted_columns.find(point.column_idx) == accepted_columns.end()) {
      continue;
    }

    voxel_grid_.markVoxelInMap(point.mx, point.my, point.mz, mark_threshold_);
  }

  publishAreaObstacleFlag(robot_x, false);
  publish_voxel_grid();
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void IntensityVoxelLayer::clusterCandidatePoints(
    const std::vector<CandidateObstaclePoint> &points,
    std::vector<std::vector<std::size_t>> &clusters, int min_points) {
  clusters.clear();

  if (points.empty()) {
    return;
  }

  const double tolerance = std::max(0.01, point_cluster_tolerance_);
  const double tolerance_sq = tolerance * tolerance;
  const int required_points = std::max(1, min_points);
  std::unordered_map<PointClusterGridKey, std::vector<std::size_t>,
                     PointClusterGridKeyHash>
      spatial_grid;

  spatial_grid.reserve(points.size() * 2);
  auto get_cluster_grid_key = [&](const CandidateObstaclePoint &point) {
    return PointClusterGridKey{
        static_cast<int>(std::floor(point.x / tolerance)),
        static_cast<int>(std::floor(point.y / tolerance)),
        static_cast<int>(std::floor(point.z / tolerance))};
  };

  for (std::size_t index = 0; index < points.size(); ++index) {
    spatial_grid[get_cluster_grid_key(points[index])].push_back(index);
  }

  std::vector<bool> visited(points.size(), false);

  for (std::size_t start_index = 0; start_index < points.size();
       ++start_index) {
    if (visited[start_index]) {
      continue;
    }

    std::vector<std::size_t> cluster;
    std::queue<std::size_t> search_queue;

    visited[start_index] = true;
    search_queue.push(start_index);

    while (!search_queue.empty()) {
      const std::size_t current_index = search_queue.front();
      search_queue.pop();

      cluster.push_back(current_index);
      const auto &current_point = points[current_index];
      const PointClusterGridKey current_key =
          get_cluster_grid_key(current_point);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const PointClusterGridKey neighbor_key{
                current_key.x + dx, current_key.y + dy, current_key.z + dz};
            const auto grid_iterator = spatial_grid.find(neighbor_key);
            if (grid_iterator == spatial_grid.end()) {
              continue;
            }

            for (const std::size_t next_index : grid_iterator->second) {
              if (visited[next_index]) {
                continue;
              }

              const auto &next_point = points[next_index];
              const double diff_x = current_point.x - next_point.x;
              const double diff_y = current_point.y - next_point.y;
              const double diff_z = current_point.z - next_point.z;
              const double distance_sq =
                  diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

              if (distance_sq > tolerance_sq) {
                continue;
              }

              visited[next_index] = true;
              search_queue.push(next_index);
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= required_points) {
      clusters.push_back(std::move(cluster));
    }
  }
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x,
                                       double new_origin_y) {
  const int cell_offset_x =
      static_cast<int>((new_origin_x - origin_x_) / resolution_);
  const int cell_offset_y =
      static_cast<int>((new_origin_y - origin_y_) / resolution_);

  if (cell_offset_x == 0 && cell_offset_y == 0) {
    return;
  }

  origin_x_ += cell_offset_x * resolution_;
  origin_y_ += cell_offset_y * resolution_;

  voxel_grid_.reset();
}

bool IntensityVoxelLayer::updateTiltSuppressionState() {
  if (!disable_dynamic_map_on_tilt_) {
    dynamic_map_suppressed_by_tilt_ = false;
    current_tilt_deg_ = 0.0;
    return false;
  }

  try {
    const auto transform =
        tf_->lookupTransform(global_frame_, tilt_frame_, tf2::TimePointZero);
    const auto &rotation = transform.transform.rotation;
    tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
    const double roll_deg = std::abs(roll) * RAD_TO_DEG;
    const double pitch_deg = std::abs(pitch) * RAD_TO_DEG;
    const double max_tilt_deg = std::max(roll_deg, pitch_deg);
    const bool previous_state = dynamic_map_suppressed_by_tilt_;

    if (!dynamic_map_suppressed_by_tilt_) {
      if (max_tilt_deg >= tilt_disable_threshold_deg_) {
        dynamic_map_suppressed_by_tilt_ = true;
      }
    } else if (roll_deg <= tilt_recover_threshold_deg_ &&
               pitch_deg <= tilt_recover_threshold_deg_) {
      dynamic_map_suppressed_by_tilt_ = false;
    }

    if (previous_state != dynamic_map_suppressed_by_tilt_) {
      if (dynamic_map_suppressed_by_tilt_) {
        RCLCPP_WARN(logger_,
                    "Vehicle tilt too large: roll=%.2f deg, pitch=%.2f deg. "
                    "Disable dynamic obstacle map.",
                    roll_deg, pitch_deg);
      } else {
        RCLCPP_INFO(logger_,
                    "Vehicle posture recovered: roll=%.2f deg, pitch=%.2f deg. "
                    "Enable dynamic obstacle map.",
                    roll_deg, pitch_deg);
      }
    }

    current_tilt_deg_ = max_tilt_deg;
  } catch (const tf2::TransformException &exception) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000, "Cannot get tilt transform %s -> %s: %s",
        global_frame_.c_str(), tilt_frame_.c_str(), exception.what());
  }

  return dynamic_map_suppressed_by_tilt_;
}

void IntensityVoxelLayer::clearDynamicObstacleHistory(double *min_x,
                                                      double *min_y,
                                                      double *max_x,
                                                      double *max_y) {
  const double expand_distance = obstacle_expand_size_ * resolution_;

  for (const auto &cell : active_obstacle_cells_) {
    touch(cell.wx - expand_distance, cell.wy - expand_distance,
          min_x, min_y, max_x, max_y);
    touch(cell.wx + expand_distance, cell.wy + expand_distance,
          min_x, min_y, max_x, max_y);
  }

  active_obstacle_cells_.clear();
  voxel_grid_.reset();
}

void IntensityVoxelLayer::clearNearDynamicObstacleHistory(
    double robot_x, double robot_y, double radius, double *min_x, double *min_y,
    double *max_x, double *max_y) {
  const double valid_radius = std::max(0.0, radius);
  const double expand_distance = obstacle_expand_size_ * resolution_;
  const double effective_clear_radius =
      valid_radius + std::sqrt(2.0) * expand_distance + 0.5 * resolution_;
  const double clear_radius_sq =
      effective_clear_radius * effective_clear_radius;

  for (auto it = active_obstacle_cells_.begin();
       it != active_obstacle_cells_.end();) {
    const double dx = it->wx - robot_x;
    const double dy = it->wy - robot_y;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq > clear_radius_sq) {
      ++it;
      continue;
    }

    touch(it->wx - expand_distance, it->wy - expand_distance,
          min_x, min_y, max_x, max_y);
    touch(it->wx + expand_distance, it->wy + expand_distance,
          min_x, min_y, max_x, max_y);

    it = active_obstacle_cells_.erase(it);
  }
}

std::uint64_t IntensityVoxelLayer::makeObstacleHistoryKey(int grid_x,
                                                          int grid_y) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(grid_x))
          << 32) |
         static_cast<std::uint32_t>(grid_y);
}

void IntensityVoxelLayer::pruneObstacleCellTemporalStates(double now_sec) {
  for (auto it = obstacle_cell_temporal_states_.begin();
       it != obstacle_cell_temporal_states_.end();) {
    if (it->second.last_observed_sec < 0.0 ||
        now_sec - it->second.last_observed_sec >
            sudden_obstacle_history_retention_) {
      it = obstacle_cell_temporal_states_.erase(it);
    } else {
      ++it;
    }
  }
}

void IntensityVoxelLayer::resetSuddenObstacleHistory() {
  obstacle_cell_temporal_states_.clear();
}

void IntensityVoxelLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                      int min_i, int min_j, int max_i,
                                      int max_j) {
  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
  constexpr double kAreaMinX = 11.5;
  constexpr double kAreaMaxX = 13.5;
  constexpr double kAreaMinY = -4.5;
  constexpr double kAreaMaxY = 2.0;
  constexpr unsigned char kAdditionalCost = 150;

  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  const double map_min_x = master_grid.getOriginX();
  const double map_min_y = master_grid.getOriginY();
  const double resolution = master_grid.getResolution();
  const int map_size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int map_size_y = static_cast<int>(master_grid.getSizeInCellsY());
  const double map_max_x =
      map_min_x + static_cast<double>(map_size_x) * resolution;
  const double map_max_y =
      map_min_y + static_cast<double>(map_size_y) * resolution;

  if (kAreaMaxX <= map_min_x || kAreaMinX >= map_max_x ||
      kAreaMaxY <= map_min_y || kAreaMinY >= map_max_y) {
    return;
  }

  const int area_start_i = std::clamp(
      static_cast<int>(std::floor((kAreaMinX - map_min_x) / resolution)), 0,
      map_size_x);
  const int area_end_i = std::clamp(
      static_cast<int>(std::ceil((kAreaMaxX - map_min_x) / resolution)), 0,
      map_size_x);
  const int area_start_j = std::clamp(
      static_cast<int>(std::floor((kAreaMinY - map_min_y) / resolution)), 0,
      map_size_y);
  const int area_end_j = std::clamp(
      static_cast<int>(std::ceil((kAreaMaxY - map_min_y) / resolution)), 0,
      map_size_y);
  const int start_i = std::max(min_i, area_start_i);
  const int end_i = std::min(max_i, area_end_i);
  const int start_j = std::max(min_j, area_start_j);
  const int end_j = std::min(max_j, area_end_j);

  if (start_i >= end_i || start_j >= end_j) {
    return;
  }

  constexpr int kMaxTraversableCost =
      static_cast<int>(INSCRIBED_INFLATED_OBSTACLE) - 1;

  for (int my = start_j; my < end_j; ++my) {
    for (int mx = start_i; mx < end_i; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(mx),
                             static_cast<unsigned int>(my), wx, wy);

      if (wx < kAreaMinX || wx > kAreaMaxX || wy < kAreaMinY ||
          wy > kAreaMaxY) {
        continue;
      }

      const unsigned char old_cost = master_grid.getCost(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my));

      if (old_cost == NO_INFORMATION ||
          old_cost >= INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }

      const int boosted_cost =
          std::min(kMaxTraversableCost, static_cast<int>(old_cost) +
                                            static_cast<int>(kAdditionalCost));

      master_grid.setCost(static_cast<unsigned int>(mx),
                          static_cast<unsigned int>(my),
                          static_cast<unsigned char>(boosted_cost));
    }
  }
}

void IntensityVoxelLayer::publishAreaObstacleFlag(double robot_x,
                                                  bool force_zero) {
  if (!area_obstacle_flag_pub_) {
    return;
  }

  std_msgs::msg::Int32 flag_msg;
  flag_msg.data = 0;

  if (force_zero) {
    area_obstacle_flag_pub_->publish(flag_msg);
    return;
  }

  const int rm_task = rm_task_.load(std::memory_order_relaxed);
  bool area_configured = false;
  double area_min_x = 0.0;
  double area_max_x = 0.0;
  double area_min_y = 0.0;
  double area_max_y = 0.0;

  if (rm_task == 1) {
    area_configured = true;
    if (robot_x >= 10.0) {
      area_min_x = 19.2;
      area_max_x = 20.1;
      area_min_y = 5.3;
      area_max_y = 6.4;

      area_configured = true;
    }
  }

  else if (rm_task == 2) {
    area_configured = true;
    if (robot_x >= 10.0) {
      area_min_x = 15.5;
      area_max_x = 16.3;
      area_min_y = 3.0;
      area_max_y = 4.2;
      area_configured = true;
    }
  }

  if (!area_configured) {
    area_obstacle_flag_pub_->publish(flag_msg);
    return;
  }
  flag_msg.data = 1;
  constexpr std::size_t kMinimumObstacleCellCount = 2;
  std::size_t obstacle_cell_count = 0;

  for (const auto &cell : active_obstacle_cells_) {
    const bool inside_area = cell.wx >= area_min_x && cell.wx <= area_max_x &&
                             cell.wy >= area_min_y && cell.wy <= area_max_y;

    if (!inside_area) {
      continue;
    }

    ++obstacle_cell_count;

    if (obstacle_cell_count >= kMinimumObstacleCellCount) {
      flag_msg.data = 2;
      break;
    }
  }

  area_obstacle_flag_pub_->publish(flag_msg);
}

bool IntensityVoxelLayer::isInForcedDynamicObstacleArea(double wx,
                                                        double wy) const {
  const bool in_upper_area = wx >= 16.6 && wx <= 19.0 && wy >= 5.2 && wy <= 6.6;
  const bool in_lower_area = wx >= 1.4 && wx <= 3.9 && wy >= -7.7 && wy <= -6.4;

  return in_upper_area || in_lower_area;
}

bool IntensityVoxelLayer::isInDynamicClearTriggerArea(double robot_x,
                                                      double robot_y) const {
  const bool in_upper_trigger =
      robot_x >= 14.6 && robot_x <= 16.7 && robot_y >= 5.1 && robot_y <= 6.6;
  const bool in_lower_trigger =
      robot_x >= 3.9 && robot_x <= 5.9 && robot_y >= -7.7 && robot_y <= -6.4;

  return in_upper_trigger || in_lower_trigger;
}

void IntensityVoxelLayer::updateForcedDynamicObstacleAreas(
    bool mark_as_obstacle, double *min_x, double *min_y, double *max_x,
    double *max_y) {
  auto process_rectangle = [&](double rect_min_x, double rect_min_y,
                               double rect_max_x, double rect_max_y) {
    const double min_wx = std::min(rect_min_x, rect_max_x);
    const double max_wx = std::max(rect_min_x, rect_max_x);
    const double min_wy = std::min(rect_min_y, rect_max_y);
    const double max_wy = std::max(rect_min_y, rect_max_y);

    touch(min_wx, min_wy, min_x, min_y, max_x, max_y);
    touch(max_wx, max_wy, min_x, min_y, max_x, max_y);

    if (!mark_as_obstacle) {
      return;
    }

    if (size_x_ == 0 || size_y_ == 0) {
      return;
    }

    const double map_max_x =
        origin_x_ + static_cast<double>(size_x_) * resolution_;
    const double map_max_y =
        origin_y_ + static_cast<double>(size_y_) * resolution_;

    if (max_wx < origin_x_ || min_wx >= map_max_x || max_wy < origin_y_ ||
        min_wy >= map_max_y) {
      return;
    }

    const int start_x = std::clamp(
        static_cast<int>(std::floor((min_wx - origin_x_) / resolution_)), 0,
        static_cast<int>(size_x_) - 1);
    const int end_x = std::clamp(
        static_cast<int>(std::floor((max_wx - origin_x_) / resolution_)), 0,
        static_cast<int>(size_x_) - 1);
    const int start_y = std::clamp(
        static_cast<int>(std::floor((min_wy - origin_y_) / resolution_)), 0,
        static_cast<int>(size_y_) - 1);
    const int end_y = std::clamp(
        static_cast<int>(std::floor((max_wy - origin_y_) / resolution_)), 0,
        static_cast<int>(size_y_) - 1);

    for (int mx = start_x; mx <= end_x; ++mx) {
      for (int my = start_y; my <= end_y; ++my) {
        double wx = 0.0;
        double wy = 0.0;

        mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
                   wx, wy);

        if (wx < min_wx || wx > max_wx || wy < min_wy || wy > max_wy) {
          continue;
        }

        costmap_[getIndex(static_cast<unsigned int>(mx),
                          static_cast<unsigned int>(my))] = LETHAL_OBSTACLE;
      }
    }
  };

  process_rectangle(17.0, 5.2, 18.3, 6.6);

  process_rectangle(1.9, -7.7, 3.6, -6.4);
}

void IntensityVoxelLayer::clearDynamicObstacleHistoryInForcedAreas(
    double *min_x, double *min_y, double *max_x, double *max_y) {
  const double expand_distance = obstacle_expand_size_ * resolution_;

  for (auto it = active_obstacle_cells_.begin();
       it != active_obstacle_cells_.end();) {
    if (!isInForcedDynamicObstacleArea(it->wx, it->wy)) {
      ++it;
      continue;
    }

    touch(it->wx - expand_distance, it->wy - expand_distance,
          min_x, min_y, max_x, max_y);
    touch(it->wx + expand_distance, it->wy + expand_distance,
          min_x, min_y, max_x, max_y);

    it = active_obstacle_cells_.erase(it);
  }
}

void IntensityVoxelLayer::clearDynamicObstacleHistoryOutsideForcedAreas(
    double *min_x, double *min_y, double *max_x, double *max_y) {
  const double expand_distance = obstacle_expand_size_ * resolution_;

  for (auto it = active_obstacle_cells_.begin();
       it != active_obstacle_cells_.end();) {
    if (isInForcedDynamicObstacleArea(it->wx, it->wy)) {
      ++it;
      continue;
    }

    touch(it->wx - expand_distance, it->wy - expand_distance,
          min_x, min_y, max_x, max_y);
    touch(it->wx + expand_distance, it->wy + expand_distance,
          min_x, min_y, max_x, max_y);

    it = active_obstacle_cells_.erase(it);
  }

  voxel_grid_.reset();
}

bool IntensityVoxelLayer::isInForcedAreaClearTriggerRegion(
    double robot_x, double robot_y) const {
  const bool in_lower_region =
      robot_x >= 0.2 && robot_x <= 3.9 && robot_y >= -7.7 && robot_y <= -6.3;
  const bool in_upper_region =
      robot_x >= 16.6 && robot_x <= 20.6 && robot_y >= 5.0 && robot_y <= 6.8;

  return in_lower_region || in_upper_region;
}
} // namespace xx_nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(xx_nav2_costmap_2d::IntensityVoxelLayer,
                       nav2_costmap_2d::Layer)
