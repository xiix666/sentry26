#ifndef PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "tf2_ros/message_filter.h"
#include <unordered_map>
#include <unordered_set>

namespace xx_nav2_costmap_2d
{

class IntensityVoxelLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  IntensityVoxelLayer() : voxel_grid_(0, 0, 0)
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class's parent class Costmap2D.
  }

  virtual ~IntensityVoxelLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);

  void updateOrigin(double new_origin_x, double new_origin_y);
  bool isDiscretized() { return true; }
  virtual void matchSize();
  virtual void reset();
  virtual bool isClearable() { return false; }

protected:
  virtual void resetMaps();
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  void markObstacleWithExpand(
    unsigned int mx, unsigned int my, double wx, double wy,
    double* min_x, double* min_y, double* max_x, double* max_y);
private:
  struct ActiveObstacle
  {
    double wx;
    double wy;
    double last_hit_time;
  };
  bool publish_voxel_;
  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  nav2_voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;
  double min_obstacle_intensity_, max_obstacle_intensity_;
  int unknown_threshold_, mark_threshold_, size_z_;
  rclcpp::Clock::SharedPtr clock_;
  double * last_hit_time_grid_ = nullptr;   // 每个体素最近一次命中时间
  double obstacle_hold_time_ = 0.5;         
  double max_gradient_threshold_ = 1.0;    
  double z_hit_range_ = 0.2; 
  std::vector<ActiveObstacle> active_obstacle_cells_;

  int * hit_count_grid_ = nullptr;  // 连续命中计数网格
  int continuous_hit_threshold_ = 2; // 连续命中阈值
  int * voxel_point_count_ = nullptr; // 单批次体素点数统计网格
  int neighborhood_radius_ = 1;      // 邻域校验半径（网格数）
  int neighborhood_min_voxels_ = 2;  // 邻域内有效体素数阈值
  int voxel_min_points_ = 2;         // 单个体素最小点数阈值
  int obstacle_expand_size_ = 1;
  
  // 内联函数声明：计算3D体素的一维索引
  inline unsigned int getVoxelIndex(unsigned int mx, unsigned int my, unsigned int mz) const
  {
    return mx + my * size_x_ + mz * size_x_ * size_y_;
  }
  inline unsigned int countNeighborhoodVoxels(unsigned int mx, unsigned int my, unsigned int mz);
  inline bool hasOtherZHitInCurrentFrame(
    unsigned int mx, unsigned int my, unsigned int mz,
    const std::unordered_set<unsigned int> & hit_voxels) const;
  inline bool worldToMap3DFloat(
    double wx, double wy, double wz, double & mx, double & my, double & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    return mx < size_x_ && my < size_y_ && mz < size_z_;
  }

  inline bool worldToMap3D(
    double wx, double wy, double wz, unsigned int & mx, unsigned int & my, unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);
    mz = static_cast<int>((wz - origin_z_) / z_resolution_);

    return mx < size_x_ && my < size_y_ && mz < size_z_;
  }

  inline void mapToWorld3D(
    unsigned int mx, unsigned int my, unsigned int mz, double & wx, double & wy, double & wz)
  {
    // returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  }
};

}  // namespace pb_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
