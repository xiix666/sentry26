#ifndef PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

namespace xx_nav2_costmap_2d
{

class IntensityVoxelLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  IntensityVoxelLayer()
  : voxel_grid_(0, 0, 0)
  {
    costmap_ = nullptr;
  }

  ~IntensityVoxelLayer() override;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateOrigin(double new_origin_x, double new_origin_y);

  // ROS 2 Humble当前父类中没有可覆盖的同签名虚函数，不能写override。
  bool isDiscretized() {return true;}

  void matchSize() override;
  void reset() override;

  // 保留原接口；不写override以兼容不同Nav2版本。
  bool isClearable() {return false;}

  void markObstacleWithExpand(
    unsigned int mx, unsigned int my,
    double wx, double wy,
    double * min_x, double * min_y,
    double * max_x, double * max_y);

  void markStaticObstacleArea(
    double robot_x, double robot_y,
    double * min_x, double * min_y,
    double * max_x, double * max_y);

protected:
  void resetMaps() override;

  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y);

private:
  struct PointClusterGridKey
  {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const PointClusterGridKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct PointClusterGridKeyHash
  {
    std::size_t operator()(const PointClusterGridKey & key) const
    {
      const std::int64_t x = static_cast<std::int64_t>(key.x);
      const std::int64_t y = static_cast<std::int64_t>(key.y);
      const std::int64_t z = static_cast<std::int64_t>(key.z);

      return static_cast<std::size_t>(
        (x * 73856093LL) ^
        (y * 19349663LL) ^
        (z * 83492791LL));
    }
  };

  struct CandidateObstaclePoint
  {
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
    // gradient >= max_gradient_threshold_
    bool hard_gradient{false};

    // low_gradient_threshold_ <= gradient < max_gradient_threshold_
    bool soft_mid_gradient{false};
  };

  struct ActiveObstacleCell
  {
    // 使用世界坐标保存，因此rolling window移动时不需要平移历史数组。
    double wx{0.0};
    double wy{0.0};
    double last_hit_time{0.0};
  };
  // 相对高度达到该值时，高度分数为1
  double height_score_full_{0.35};
  void clusterCandidatePoints(
    const std::vector<CandidateObstaclePoint> & points,
    std::vector<std::vector<std::size_t>> & clusters,
    int min_points);

  bool updateTiltSuppressionState();

  void clearDynamicObstacleHistory(
    double * min_x, double * min_y,
    double * max_x, double * max_y);
  void clearNearDynamicObstacleHistory(
    double robot_x,
    double robot_y,
    double radius,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  static double clamp01(double value)
  {
    return std::clamp(value, 0.0, 1.0);
  }

  inline bool worldToMap3D(
    double wx, double wy, double wz,
    unsigned int & mx, unsigned int & my, unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

    return
      mx < size_x_ &&
      my < size_y_ &&
      mz < static_cast<unsigned int>(size_z_);
  }

  // --------------------------------------------------------------------------
  // 原有硬过滤和梯度阈值
  // --------------------------------------------------------------------------
  // intensity不是反射强度，而是地形分析得到的相对局部平面高度。
  double min_obstacle_intensity_{0.1};
  double max_obstacle_intensity_{2.0};

  double low_gradient_threshold_{0.2};
  double max_gradient_threshold_{1.0};

  // --------------------------------------------------------------------------
  // 聚类、硬通道、软通道
  // --------------------------------------------------------------------------
  double point_cluster_tolerance_{0.15};

  // 原硬通道：簇总点数要求。
  int min_cluster_points_{8};

  // 硬簇中至少需要多少个 gradient >= max 的点。
  int cluster_hard_evidence_min_points_{6};

  // 为了让小而真实的中梯度障碍也能参与评分，聚类最低点数可低于硬簇点数。
  int soft_min_cluster_points_{3};

  // 车辆平稳时使用的基础软簇阈值。
  double soft_score_threshold_{0.55};

  // 近距离只作为软簇评分加分，不再直接成为硬障碍。
  double near_obstacle_radius_{0.6};

  // 达到此速度时，速度可信度降到0；只会提高软阈值。
  double speed_low_confidence_mps_{4.0};

  // --------------------------------------------------------------------------
  // 障碍保持
  // --------------------------------------------------------------------------
  double obstacle_hold_time_{1.5};
  std::vector<ActiveObstacleCell> active_obstacle_cells_;

  // --------------------------------------------------------------------------
  // 图层与体素配置
  // --------------------------------------------------------------------------
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

  // --------------------------------------------------------------------------
  // 固定障碍区域
  // --------------------------------------------------------------------------
  bool use_static_obstacle_area_{false};
  double static_obs_min_x_{0.0};
  double static_obs_min_y_{0.0};
  double static_obs_max_x_{0.0};
  double static_obs_max_y_{0.0};
  double static_obs2_min_x_{0.0};
  double static_obs2_min_y_{0.0};
  double static_obs2_max_x_{0.0};
  double static_obs2_max_y_{0.0};

  // --------------------------------------------------------------------------
  // 倾角保护
  // --------------------------------------------------------------------------
  bool disable_dynamic_map_on_tilt_{true};
  std::string tilt_frame_{"base_link"};
  double tilt_disable_threshold_deg_{20.0};
  double tilt_recover_threshold_deg_{15.0};
  bool dynamic_map_suppressed_by_tilt_{false};
  double current_roll_deg_{0.0};
  double current_pitch_deg_{0.0};
  double current_tilt_deg_{0.0};

  // --------------------------------------------------------------------------
  // Point-LIO速度
  // --------------------------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  std::string velocity_odom_topic_{"/aft_mapped_to_init"};
  std::atomic<double> current_speed_mps_{0.0};

  // 比赛任务状态
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rm_task_sub_;
  std::atomic<int> rm_task_{0};

  // 加速度近场抑制参数
  double acceleration_suppression_threshold_mps2_{3.0};
  double acceleration_suppression_radius_{2.0};
  // 加速度触发后，近场感知继续关闭的时间
  double acceleration_suppression_hold_time_{0.8};

  // 近场加速度抑制截止时刻
  std::atomic<double> acceleration_suppression_until_sec_{-1.0};
  // 根据里程计速度差计算的平面加速度
  std::atomic<double> current_acceleration_mps2_{0.0};

  // 最近一次收到里程计的本地时间，用于避免里程计断流后一直抑制
  std::atomic<double> acceleration_update_time_sec_{-1.0};

  // 以下变量只在里程计回调中访问
  bool has_last_odom_speed_{false};
  double last_odom_speed_mps_{0.0};
  double last_odom_stamp_sec_{-1.0};

  
};

}  // namespace xx_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_