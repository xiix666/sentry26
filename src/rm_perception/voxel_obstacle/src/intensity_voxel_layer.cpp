#include "intensity_voxel_layer.hpp"

#include <vector>
#include <cstring>
#include <unordered_set>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#define VOXEL_BITS 16

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;

namespace xx_nav2_costmap_2d
{

void IntensityVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ =
    node->get_parameter(name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(name_ + ".max_obstacle_height").as_double();
  combination_method_ = node->get_parameter(name_ + ".combination_method").as_int();

  obstacle_hold_time_ = node->declare_parameter(name_ + ".obstacle_hold_time", 0.2);
  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  min_obstacle_intensity_ = node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  max_obstacle_intensity_ = node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  z_resolution_ = node->declare_parameter(name_ + ".z_resolution", 0.05);
  unknown_threshold_ =
    node->declare_parameter(name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  mark_threshold_ = node->declare_parameter(name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(name_ + ".publish_voxel_map", false);
  
  // 核心参数
  continuous_hit_threshold_ = node->declare_parameter(name_ + ".continuous_hit_threshold", 2);
  // 新增：动态邻域配置参数
  neighborhood_radius_ = node->declare_parameter(name_ + ".neighborhood_radius", 1); // 邻域半径（网格数）
  neighborhood_min_voxels_ = node->declare_parameter(name_ + ".neighborhood_min_voxels", 1); // 邻域有效体素阈值

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
}

IntensityVoxelLayer::~IntensityVoxelLayer() 
{
  // 释放计数网格内存
  if (hit_count_grid_ != nullptr) {
    delete[] hit_count_grid_;
    hit_count_grid_ = nullptr;
  }
  if (last_hit_time_grid_ != nullptr) {
    delete[] last_hit_time_grid_;
    last_hit_time_grid_ = nullptr;
  }
}

void IntensityVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto & i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  
  // 初始化连续命中计数网格（x*y*z维度）
  unsigned int total_voxels = size_x_ * size_y_ * size_z_;
  if (hit_count_grid_ != nullptr) {
    delete[] hit_count_grid_;
  }
  hit_count_grid_ = new unsigned int[total_voxels](); // 初始化为0
  if (last_hit_time_grid_ != nullptr) {
    delete[] last_hit_time_grid_;
  }
  last_hit_time_grid_ = new double[total_voxels];
  for (unsigned int i = 0; i < total_voxels; ++i) {
    last_hit_time_grid_[i] = -1.0;
  }
}

// 获取体素在网格中的索引
// inline unsigned int IntensityVoxelLayer::getVoxelIndex(unsigned int mx, unsigned int my, unsigned int mz)
// {
//   return mx + my * size_x_ + mz * size_x_ * size_y_;
// }

// 动态邻域校验：根据配置的半径统计有效体素数
inline unsigned int IntensityVoxelLayer::countNeighborhoodVoxels(unsigned int mx, unsigned int my, unsigned int mz)
{
  unsigned int count = 0;
  // 遍历动态邻域范围：[-radius, +radius]
  for (int dx = -static_cast<int>(neighborhood_radius_); dx <= static_cast<int>(neighborhood_radius_); ++dx) {
    for (int dy = -static_cast<int>(neighborhood_radius_); dy <= static_cast<int>(neighborhood_radius_); ++dy) {
      // 跳过当前体素自身
      if (dx == 0 && dy == 0) continue;
      
      // 检查邻域体素是否在网格范围内（防止越界）
      unsigned int nx = mx + dx;
      unsigned int ny = my + dy;
      if (nx >= size_x_ || ny >= size_y_) continue;

      // 检查邻域体素是否有有效命中（连续命中计数>0）
      unsigned int n_idx = getVoxelIndex(nx, ny, mz);
      if (hit_count_grid_[n_idx] > 0) {
        count++;
      }
    }
  }
  return count;
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
  active_obstacle_cells_.clear();
}

void IntensityVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
  // active_obstacle_cells_.clear();
  // 重置计数网格（保留历史计数，仅清空costmap）
}

void IntensityVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  resetMaps();
  if (!enabled_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;
  current_ = current;

  // 临时记录本次命中的体素
  std::unordered_set<unsigned int> hit_voxels;
  double now_sec = clock_->now().seconds();
  // 第一步：遍历点云，更新体素命中计数
  for (const auto & obs : observations) {
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*obs.cloud_, "intensity");
    
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      double px = *it_x, py = *it_y, pz = *it_z;
      unsigned int mx, my, mz;
      if (pz < origin_z_) {
        if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) continue;
      } else if (!worldToMap3D(px, py, pz, mx, my, mz)) continue;
      unsigned int voxel_idx = getVoxelIndex(mx, my, mz);

      // 原有过滤逻辑（高度、强度、距离）
      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) continue;
      if (*it_i < min_obstacle_intensity_ || *it_i > max_obstacle_intensity_) continue;
      double sq_dist = (px - obs.origin_.x)*(px - obs.origin_.x) + (py - obs.origin_.y)*(py - obs.origin_.y) + (pz - obs.origin_.z)*(pz - obs.origin_.z);
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) continue;

      // 增加连续命中计数
      hit_count_grid_[voxel_idx]++;
      hit_voxels.insert(voxel_idx);

      // 标记体素到voxel_grid（原有逻辑）
      voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_);
    }
  }
  double sec = clock_->now().seconds();
  // 第二步：过滤孤立体素，仅标记非孤立且连续命中达标的体素为障碍
  for (const auto & voxel_idx : hit_voxels) {
    // 解析体素坐标
    unsigned int mz = voxel_idx / (size_x_ * size_y_);
    unsigned int rem = voxel_idx % (size_x_ * size_y_);
    unsigned int my = rem / size_x_;
    unsigned int mx = rem % size_x_;

    // 核心：动态邻域校验（根据配置的半径统计有效体素）
    unsigned int neighbor_count = countNeighborhoodVoxels(mx, my, mz);
    if (neighbor_count < neighborhood_min_voxels_) {
      continue; // 邻域有效体素不足，跳过障碍标记
    }
    bool has_other_z_hit = false;
    int z_range = static_cast<int>(std::ceil(0.5 / z_resolution_));

    for (int dz = -z_range; dz <= z_range; ++dz) {
      if (dz == 0) continue;  // 跳过自己这一层

      int nz = static_cast<int>(mz) + dz;
      if (nz < 0 || nz >= static_cast<int>(size_z_)) {
        continue;
      }

      unsigned int neighbor_voxel_idx = getVoxelIndex(mx, my, static_cast<unsigned int>(nz));
      if (hit_voxels.find(neighbor_voxel_idx) != hit_voxels.end()) {
        has_other_z_hit = true;
        break;
      }
    }

    // if (!has_other_z_hit) {
    //   continue;  // z方向0.2m内没有其他点云命中，跳过障碍标记
    // }
    // 只有连续命中次数≥阈值 + 非孤立体素，才标记为致命障碍

    // last_hit_time_grid_[voxel_idx] = sec; 
    if (hit_count_grid_[voxel_idx] >= continuous_hit_threshold_) {
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      double wx, wy;
      mapToWorld(mx, my, wx, wy);
      active_obstacle_cells_.push_back({wx, wy, sec});
      touch(wx, wy, min_x, min_y, max_x, max_y);
    }
  }

  // 重置未命中体素的计数
  unsigned int total_voxels = size_x_ * size_y_ * size_z_;
  for (unsigned int i = 0; i < total_voxels; ++i) {
    if (hit_voxels.find(i) == hit_voxels.end()) {
      hit_count_grid_[i] = 0;
    }
  }
  for (auto it = active_obstacle_cells_.begin(); it != active_obstacle_cells_.end(); ) {
    if (now_sec - it->last_hit_time > obstacle_hold_time_) {
      it = active_obstacle_cells_.erase(it);
      continue;
    }
  
    unsigned int mx, my;
    if (worldToMap(it->wx, it->wy, mx, my)) {
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(it->wx, it->wy, min_x, min_y, max_x, max_y);
    }
  
    ++it;
  }
  // 发布体素网格（原有逻辑）
  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();
    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
  // active_obstacle_cells_.clear();
  matchSize();
}

}  // namespace pb_nav2_costmap_2d

// 头文件新增声明（关键！）
/*
private:
  unsigned int * hit_count_grid_ = nullptr;  // 连续命中计数网格
  unsigned int continuous_hit_threshold_ = 2; // 连续命中阈值
  unsigned int neighborhood_radius_ = 1;      // 动态邻域半径（网格数），1=3x3邻域，2=5x5邻域...
  unsigned int neighborhood_min_voxels_ = 1;  // 邻域内有效体素数阈值

  inline unsigned int getVoxelIndex(unsigned int mx, unsigned int my, unsigned int mz);
  inline unsigned int countNeighborhoodVoxels(unsigned int mx, unsigned int my, unsigned int mz);
*/

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(xx_nav2_costmap_2d::IntensityVoxelLayer, nav2_costmap_2d::Layer)