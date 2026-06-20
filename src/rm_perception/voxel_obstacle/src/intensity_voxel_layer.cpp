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
  
    node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
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
    node->get_parameter(name_ + ".min_obstacle_intensity", min_obstacle_intensity_);
  
    node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
    node->get_parameter(name_ + ".max_obstacle_intensity", max_obstacle_intensity_);
  
    node->declare_parameter(name_ + ".z_resolution", 0.05);
    node->get_parameter(name_ + ".z_resolution", z_resolution_);
  
    node->declare_parameter(name_ + ".unknown_threshold", 15);
    node->get_parameter(name_ + ".unknown_threshold", unknown_threshold_);
    unknown_threshold_ += (VOXEL_BITS - size_z_);
  
    node->declare_parameter(name_ + ".mark_threshold", 0);
    node->get_parameter(name_ + ".mark_threshold", mark_threshold_);
  
    node->declare_parameter(name_ + ".publish_voxel_map", false);
    node->get_parameter(name_ + ".publish_voxel_map", publish_voxel_);
  
    node->declare_parameter(name_ + ".max_gradient_threshold", 1.0);
    node->get_parameter(name_ + ".max_gradient_threshold", max_gradient_threshold_);
  
    node->declare_parameter(name_ + ".continuous_hit_threshold", 2);
    node->get_parameter(name_ + ".continuous_hit_threshold", continuous_hit_threshold_);
  
    node->declare_parameter(name_ + ".neighborhood_radius", 1);
    node->get_parameter(name_ + ".neighborhood_radius", neighborhood_radius_);
  
    node->declare_parameter(name_ + ".neighborhood_min_voxels", 1);
    node->get_parameter(name_ + ".neighborhood_min_voxels", neighborhood_min_voxels_);
  
    node->declare_parameter(name_ + ".obstacle_expand_size", 1);  // 膨胀格子数 1=3x3, 2=5x5
    node->get_parameter(name_ + ".obstacle_expand_size", obstacle_expand_size_);

    node->declare_parameter(name_ + ".near_obstacle_radius", 0.6);
    node->get_parameter(name_ + ".near_obstacle_radius", near_obstacle_radius_);

    node->declare_parameter(name_ + ".low_gradient_threshold", 0.2);
    node->get_parameter(name_ + ".low_gradient_threshold", low_gradient_threshold_);
    
    node->declare_parameter(name_ + ".use_static_obstacle_area", false);
    node->get_parameter(name_ + ".use_static_obstacle_area", use_static_obstacle_area_);

    node->declare_parameter(name_ + ".static_obs_min_x", 0.0);
    node->declare_parameter(name_ + ".static_obs_min_y", 0.0);
    node->declare_parameter(name_ + ".static_obs_max_x", 0.0);
    node->declare_parameter(name_ + ".static_obs_max_y", 0.0);
    node->declare_parameter(name_ + ".static_obs2_min_x", 0.0);
    node->declare_parameter(name_ + ".static_obs2_min_y", 0.0);
    node->declare_parameter(name_ + ".static_obs2_max_x", 0.0);
    node->declare_parameter(name_ + ".static_obs2_max_y", 0.0);
    node->get_parameter(name_ + ".static_obs2_min_x", static_obs2_min_x_);
    node->get_parameter(name_ + ".static_obs2_min_y", static_obs2_min_y_);
    node->get_parameter(name_ + ".static_obs2_max_x", static_obs2_max_x_);
    node->get_parameter(name_ + ".static_obs2_max_y", static_obs2_max_y_);
    node->get_parameter(name_ + ".static_obs_min_x", static_obs_min_x_);
    node->get_parameter(name_ + ".static_obs_min_y", static_obs_min_y_);
    node->get_parameter(name_ + ".static_obs_max_x", static_obs_max_x_);
    node->get_parameter(name_ + ".static_obs_max_y", static_obs_max_y_);
    if (publish_voxel_) {
      voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
    }
    rm_task_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/rm_task",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Int32::SharedPtr msg)
    {
      rm_task_.store(msg->data, std::memory_order_relaxed);
    });
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
  int total_voxels = size_x_ * size_y_ * size_z_;
  if (hit_count_grid_ != nullptr) {
    delete[] hit_count_grid_;
  }
  hit_count_grid_ = new int[total_voxels](); // 初始化为0
  if (last_hit_time_grid_ != nullptr) {
    delete[] last_hit_time_grid_;
  }
  last_hit_time_grid_ = new double[total_voxels];
  for (int i = 0; i < total_voxels; ++i) {
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
void IntensityVoxelLayer::markStaticObstacleArea(
  double robot_x, double robot_y,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!use_static_obstacle_area_) {
    return;
  }

  const double x1 = 8.784;
  const double y1 = 5.488;
  const double x2 = 5.861;
  const double y2 = 1.669;

  const double px_limit = 6.021;
  const double py_limit = -5.304;

  const double vx = x1 - x2;
  const double vy = y1 - y2;

  const double qx = robot_x - x2;
  const double qy = robot_y - y2;

  const double cross = vx * qy - vy * qx;

  const bool left_down_of_line = cross >= 0.0;
  const bool left_down_of_point =
    robot_x <= px_limit &&
    robot_y >= py_limit;

  const bool robot_in_trigger_region =
    left_down_of_line || left_down_of_point;
  const double block_px = 13.752;
  const double block_py = 4.439;

  const bool right_up_of_block_point =
    robot_x >= block_px &&
    robot_y <= block_py;

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
    right_up_of_block_point ||
    right_up_of_block_line;
  if (!robot_in_trigger_region && !robot_in_block_region) {
    return;
  }
  auto touch_rect_area = [&](
    double rect_min_x, double rect_min_y,
    double rect_max_x, double rect_max_y)
  {
    double min_wx = std::min(rect_min_x, rect_max_x);
    double max_wx = std::max(rect_min_x, rect_max_x);
    double min_wy = std::min(rect_min_y, rect_max_y);
    double max_wy = std::max(rect_min_y, rect_max_y);

    touch(min_wx, min_wy, min_x, min_y, max_x, max_y);
    touch(max_wx, max_wy, min_x, min_y, max_x, max_y);
  };

  if (rm_task_.load(std::memory_order_relaxed) == 2) {
    touch_rect_area(
      static_obs_min_x_, static_obs_min_y_,
      static_obs_max_x_, static_obs_max_y_);

    touch_rect_area(
      static_obs2_min_x_, static_obs2_min_y_,
      static_obs2_max_x_, static_obs2_max_y_);

    return;
  }
  auto mark_rect_obstacle = [&](
    double rect_min_x, double rect_min_y,
    double rect_max_x, double rect_max_y)
  {
    double min_wx = std::min(rect_min_x, rect_max_x);
    double max_wx = std::max(rect_min_x, rect_max_x);
    double min_wy = std::min(rect_min_y, rect_max_y);
    double max_wy = std::max(rect_min_y, rect_max_y);

    unsigned int min_mx, min_my, max_mx, max_my;

    if (!worldToMap(min_wx, min_wy, min_mx, min_my)) {
      return;
    }

    if (!worldToMap(max_wx, max_wy, max_mx, max_my)) {
      return;
    }

    unsigned int start_x = std::min(min_mx, max_mx);
    unsigned int end_x   = std::max(min_mx, max_mx);
    unsigned int start_y = std::min(min_my, max_my);
    unsigned int end_y   = std::max(min_my, max_my);

    for (unsigned int mx = start_x; mx <= end_x; ++mx) {
      for (unsigned int my = start_y; my <= end_y; ++my) {
        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
      }
    }

    touch(min_wx, min_wy, min_x, min_y, max_x, max_y);
    touch(max_wx, max_wy, min_x, min_y, max_x, max_y);
  };
  mark_rect_obstacle(
    static_obs_min_x_, static_obs_min_y_,
    static_obs_max_x_, static_obs_max_y_);
  mark_rect_obstacle(
    static_obs2_min_x_, static_obs2_min_y_,
    static_obs2_max_x_, static_obs2_max_y_);

}
void IntensityVoxelLayer::markObstacleWithExpand(
  unsigned int mx, unsigned int my, double wx, double wy,
  double* min_x, double* min_y, double* max_x, double* max_y)
{

  for (int dx = -obstacle_expand_size_; dx <= obstacle_expand_size_; dx++) {
    for (int dy = -obstacle_expand_size_; dy <= obstacle_expand_size_; dy++) {
      int cx = static_cast<int>(mx) + dx;
      int cy = static_cast<int>(my) + dy;

      if (cx < 0 || cx >= static_cast<int>(size_x_) || cy < 0 || cy >= static_cast<int>(size_y_))
        continue;

      unsigned int idx = getIndex(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy));
      costmap_[idx] = LETHAL_OBSTACLE;
    }
  }

  // touch(wx, wy, min_x, min_y, max_x, max_y);
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
  markStaticObstacleArea(robot_x, robot_y, min_x, min_y, max_x, max_y);
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;
  current_ = current;

  // 临时记录本次命中的体素
  std::unordered_set<unsigned int> hit_voxels;
  double now_sec = clock_->now().seconds();

  for (const auto & obs : observations) {
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*obs.cloud_, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> it_grad(*obs.cloud_, "curvature");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_grad) {
      double px = *it_x, py = *it_y, pz = *it_z;
      unsigned int mx, my, mz;
      if (pz < origin_z_) {
        if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) continue;
      } else if (!worldToMap3D(px, py, pz, mx, my, mz)) continue;
      unsigned int voxel_idx = getVoxelIndex(mx, my, mz);

      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) continue;
      if (*it_i < min_obstacle_intensity_ || *it_i > max_obstacle_intensity_) continue;
      double sq_dist = (px - obs.origin_.x)*(px - obs.origin_.x) + (py - obs.origin_.y)*(py - obs.origin_.y) + (pz - obs.origin_.z)*(pz - obs.origin_.z);
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) continue;
      double gradient = 0.0;
  
      // bool has_gradient_field = (it_grad != it_grad.end());
      // if (has_gradient_field) {
      gradient = *it_grad;
      bool valid_gradient = std::isfinite(gradient) && gradient >= 0.0;
        // std::cout << "Point (" << px << ", " << py << ", " << pz << ") has gradient: " << gradient <<  "  " << max_gradient_threshold_ << std::endl;
      double robot_dist = std::hypot(px - robot_x, py - robot_y);
      bool gradient_obstacle = valid_gradient && gradient > max_gradient_threshold_;
      // if (gradient <= max_gradient_threshold_) continue;
      bool near_robot_obstacle = false;
      if (!gradient_obstacle) {
        near_robot_obstacle =
          robot_dist <= near_obstacle_radius_ &&
          pz >= min_obstacle_height_ &&
          pz <= max_obstacle_height_ &&
          *it_i > 0.3;
      }
      bool low_gradient_height_obstacle = false;
      if (!gradient_obstacle && !near_robot_obstacle) {
        low_gradient_height_obstacle =
          valid_gradient &&
          gradient >= low_gradient_threshold_ && gradient <= max_gradient_threshold_ &&
          *it_i > 0.3 &&
          *it_i < max_obstacle_intensity_ &&
          pz >= 0.3 &&
          pz <= 0.8;
      }
      if (!gradient_obstacle && !near_robot_obstacle && !low_gradient_height_obstacle) {
        continue;
      }

      hit_count_grid_[voxel_idx]++;
      hit_voxels.insert(voxel_idx);

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
    int z_range = static_cast<int>(std::ceil(0.2 / z_resolution_));

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

    last_hit_time_grid_[voxel_idx] = sec; 
    if (hit_count_grid_[voxel_idx] >= continuous_hit_threshold_) {
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      double wx, wy;
      mapToWorld(mx, my, wx, wy);
        bool found = false;
      for (auto& cell : active_obstacle_cells_) {
        if (std::hypot(cell.wx - wx, cell.wy - wy) < resolution_ * 0.5) {
          // std::cout << "Update obstacle cell (" << wx << ", " << wy << ") last hit time to " << cell.last_hit_time << "   " << sec <<"  ";
          cell.last_hit_time = sec;  
          // std::cout << "New last hit time: " << cell.last_hit_time << std::endl;
          found = true;
          break;
        }
      }
      if (!found) {
        active_obstacle_cells_.push_back({wx, wy, sec});
      }
      // active_obstacle_cells_.push_back({wx, wy, sec});
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
  // for (auto it = active_obstacle_cells_.begin(); it != active_obstacle_cells_.end(); ) {
  //   if (now_sec - it->last_hit_time > obstacle_hold_time_) {
  //     it = active_obstacle_cells_.erase(it);
  //     continue;
  //   }
  //   // std::cout << now_sec << std::endl;
  //   // std::cout << it->last_hit_time << std::endl;
  //   unsigned int mx, my;
  //   if (worldToMap(it->wx, it->wy, mx, my)) {
  //     unsigned int index = getIndex(mx, my);
  //     costmap_[index] = LETHAL_OBSTACLE;
  //     touch(it->wx, it->wy, min_x, min_y, max_x, max_y);
  //   }
  
  //   ++it;
  // }
  for (auto it = active_obstacle_cells_.begin(); it != active_obstacle_cells_.end(); ) {
    if (now_sec - it->last_hit_time > obstacle_hold_time_) {
      it = active_obstacle_cells_.erase(it);
      continue;
    }
  
    unsigned int mx, my;
    if (worldToMap(it->wx, it->wy, mx, my)) {
      markObstacleWithExpand(
        mx, my,
        it->wx, it->wy,
        min_x, min_y, max_x, max_y);
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