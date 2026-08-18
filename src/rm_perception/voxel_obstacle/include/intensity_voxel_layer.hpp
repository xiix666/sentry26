#ifndef PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace xx_nav2_costmap_2d {
class IntensityVoxelLayer : public nav2_costmap_2d::ObstacleLayer {
public:
  IntensityVoxelLayer() : voxel_grid_(0, 0, 0) { costmap_ = nullptr; }

  ~IntensityVoxelLayer() override;

  void onInitialize() override;

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double *min_x, double *min_y, double *max_x,
                    double *max_y) override;

  void updateOrigin(double new_origin_x, double new_origin_y);
  bool isDiscretized() { return true; }

  void matchSize() override;
  void reset() override;
  bool isClearable() { return false; }

  void markObstacleWithExpand(unsigned int mx, unsigned int my, double wx,
                              double wy, double *min_x, double *min_y,
                              double *max_x, double *max_y);

  void markStaticObstacleArea(double robot_x, double robot_y, double *min_x,
                              double *min_y, double *max_x, double *max_y);
  void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                   int min_j, int max_i, int max_j) override;

protected:
  void resetMaps() override;

  void updateFootprint(double robot_x, double robot_y, double robot_yaw,
                       double *min_x, double *min_y, double *max_x,
                       double *max_y);

private:
  struct PointClusterGridKey {
    int x{0};
    int y{0};
    int z{0};
    bool operator==(const PointClusterGridKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct PointClusterGridKeyHash {
    std::size_t operator()(const PointClusterGridKey &key) const {
      const std::int64_t x = static_cast<std::int64_t>(key.x);
      const std::int64_t y = static_cast<std::int64_t>(key.y);
      const std::int64_t z = static_cast<std::int64_t>(key.z);

      return static_cast<std::size_t>((x * 73856093LL) ^ (y * 19349663LL) ^
                                      (z * 83492791LL));
    }
  };

  struct CandidateObstaclePoint {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double gradient{0.0};
    double range{0.0};
    unsigned int mx{0};
    unsigned int my{0};
    unsigned int mz{0};
    unsigned int column_idx{0};
    double relative_height{0.0};
    bool hard_gradient{false};
    bool soft_mid_gradient{false};
  };

  struct ActiveObstacleCell {
    double wx{0.0};
    double wy{0.0};
    double last_hit_time{0.0};
  };

  struct ObstacleHistoryFrame {
    double stamp_sec{0.0};
    std::unordered_set<std::uint64_t> keys;
  };

  void clusterCandidatePoints(const std::vector<CandidateObstaclePoint> &points,
                              std::vector<std::vector<std::size_t>> &clusters,
                              int min_points);
  bool updateTiltSuppressionState();

  void clearDynamicObstacleHistory(double *min_x, double *min_y, double *max_x,
                                   double *max_y);

  void clearNearDynamicObstacleHistory(double robot_x, double robot_y,
                                       double radius, double *min_x,
                                       double *min_y, double *max_x,
                                       double *max_y);

  void publishAreaObstacleFlag(double robot_x, bool force_zero);
  static double clamp01(double value) { return std::clamp(value, 0.0, 1.0); }

  inline bool worldToMap3D(double wx, double wy, double wz, unsigned int &mx,
                           unsigned int &my, unsigned int &mz) {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

    return mx < size_x_ && my < size_y_ &&
           mz < static_cast<unsigned int>(size_z_);
  }

  bool isInForcedDynamicObstacleArea(double wx, double wy) const;
  bool isInDynamicClearTriggerArea(double robot_x, double robot_y) const;

  void updateForcedDynamicObstacleAreas(bool mark_as_obstacle, double *min_x,
                                        double *min_y, double *max_x,
                                        double *max_y);

  void clearDynamicObstacleHistoryInForcedAreas(double *min_x, double *min_y,
                                                double *max_x, double *max_y);

  void clearDynamicObstacleHistoryOutsideForcedAreas(double *min_x,
                                                     double *min_y,
                                                     double *max_x,
                                                     double *max_y);
  bool isInForcedAreaClearTriggerRegion(double robot_x, double robot_y) const;
  double min_obstacle_intensity_{0.1};
  double max_obstacle_intensity_{2.0};
  double low_gradient_threshold_{0.2};
  double max_gradient_threshold_{1.0};
  double point_cluster_tolerance_{0.15};
  int min_cluster_points_{8};
  int cluster_hard_evidence_min_points_{6};
  int soft_min_cluster_points_{3};
  double soft_score_threshold_{0.55};
  double near_obstacle_radius_{0.6};
  double speed_low_confidence_mps_{4.0};
  double height_score_full_{0.35};
  double obstacle_hold_time_{0.5};
  std::vector<ActiveObstacleCell> active_obstacle_cells_;
  bool publish_voxel_{false};
  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  nav2_voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_{0.05};
  double origin_z_{16.0};
  int unknown_threshold_{15};
  int mark_threshold_{0};
  int size_z_{16};
  int obstacle_expand_size_{1};
  rclcpp::Clock::SharedPtr clock_;
  bool use_static_obstacle_area_{false};
  double static_obs_min_x_{0.0};
  double static_obs_min_y_{0.0};
  double static_obs_max_x_{0.0};
  double static_obs_max_y_{0.0};
  double static_obs2_min_x_{0.0};
  double static_obs2_min_y_{0.0};
  double static_obs2_max_x_{0.0};
  double static_obs2_max_y_{0.0};
  bool disable_dynamic_map_on_tilt_{true};
  std::string tilt_frame_{"base_link"};
  double tilt_disable_threshold_deg_{20.0};
  double tilt_recover_threshold_deg_{15.0};
  bool dynamic_map_suppressed_by_tilt_{false};
  double current_roll_deg_{0.0};
  double current_pitch_deg_{0.0};
  double current_tilt_deg_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  std::string velocity_odom_topic_{"/aft_mapped_to_init"};
  std::atomic<double> current_speed_mps_{0.0};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rm_task_sub_;
  std::atomic<int> rm_task_{0};
  double acceleration_suppression_threshold_mps2_{3.0};
  double acceleration_suppression_radius_{2.0};
  double acceleration_suppression_hold_time_{0.8};
  std::atomic<double> acceleration_suppression_until_sec_{-1.0};
  std::atomic<double> current_acceleration_mps2_{0.0};
  std::atomic<double> acceleration_update_time_sec_{-1.0};
  bool has_last_odom_speed_{false};
  double last_odom_speed_mps_{0.0};
  double last_odom_stamp_sec_{-1.0};

  struct ObstacleCellTemporalState {
    double last_observed_sec{-1.0};
    double free_since_sec{-1.0};
    bool occupied{false};
  };
  bool enable_sudden_obstacle_history_filter_{true};
  double sudden_obstacle_near_radius_{2.5};
  double sudden_obstacle_stable_free_duration_{1.0};
  double sudden_obstacle_max_observation_gap_{0.2};
  int sudden_obstacle_rising_cell_threshold_{30};
  bool task_clear{true};
  double sudden_obstacle_history_retention_{5.0};
  bool publish_pass_enable_{false};
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr area_obstacle_flag_pub_;
  std::unordered_map<std::uint64_t, ObstacleCellTemporalState>
      obstacle_cell_temporal_states_;
  static std::uint64_t makeObstacleHistoryKey(int grid_x, int grid_y);

  void pruneObstacleCellTemporalStates(double now_sec);

  void resetSuddenObstacleHistory();
};
} // namespace xx_nav2_costmap_2d

#endif
