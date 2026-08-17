#include "static_xx.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace xx_nav2_costmap_2d
{

void xxStaticLayer::onInitialize()
{
  // 订阅/map并初始化原始StaticLayer
  nav2_costmap_2d::StaticLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error(
      "xxStaticLayer: failed to lock lifecycle node");
  }

  declareParameter(
    "clear_region_enabled",
    rclcpp::ParameterValue(true));

  declareParameter(
    "clear_min_x",
    rclcpp::ParameterValue(1.5));

  declareParameter(
    "clear_max_x",
    rclcpp::ParameterValue(4.5));

  declareParameter(
    "clear_min_y",
    rclcpp::ParameterValue(-1.8));

  declareParameter(
    "clear_max_y",
    rclcpp::ParameterValue(0.8));

  declareParameter(
    "rm_task_topic",
    rclcpp::ParameterValue(
      std::string("/rm_task")));

  declareParameter(
    "clear_task_value",
    rclcpp::ParameterValue(3));

  node->get_parameter(
    name_ + ".clear_region_enabled",
    clear_region_enabled_);

  node->get_parameter(
    name_ + ".clear_min_x",
    clear_min_x_);

  node->get_parameter(
    name_ + ".clear_max_x",
    clear_max_x_);

  node->get_parameter(
    name_ + ".clear_min_y",
    clear_min_y_);

  node->get_parameter(
    name_ + ".clear_max_y",
    clear_max_y_);

  node->get_parameter(
    name_ + ".rm_task_topic",
    rm_task_topic_);

  node->get_parameter(
    name_ + ".clear_task_value",
    clear_task_value_);

  if (clear_min_x_ > clear_max_x_) {
    std::swap(clear_min_x_, clear_max_x_);
  }

  if (clear_min_y_ > clear_max_y_) {
    std::swap(clear_min_y_, clear_max_y_);
  }

  rm_task_sub_ =
    node->create_subscription<std_msgs::msg::Int32>(
      rm_task_topic_,
      rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int32::SharedPtr msg)
      {
        const int fortress_value =
          fortress_enable_value_.load();

        const int old_rm_task =
          rm_task_value_.exchange(msg->data);

        const bool old_clear_active =
          old_rm_task == clear_task_value_ ||
          fortress_value != 0;

        const bool new_clear_active =
          msg->data == clear_task_value_ ||
          fortress_value != 0;

        if (old_clear_active != new_clear_active) {
          {
            std::lock_guard<
              nav2_costmap_2d::Costmap2D::mutex_t>
              lock(*getMutex());

            has_updated_data_ = true;
            current_ = false;
          }

          // RCLCPP_INFO(
          //   logger_,
          //   "rm_task=%d fortress_enable=%d: "
          //   "static clear region %s",
          //   msg->data,
          //   fortress_value,
          //   new_clear_active ?
          //   "ENABLED" : "DISABLED");
        }
      });
  fortress_enable_sub_ =
    node->create_subscription<std_msgs::msg::Int32>(
      fortress_enable_topic_,
      rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int32::SharedPtr msg)
      {
        const int rm_task =
          rm_task_value_.load();

        const int old_fortress =
          fortress_enable_value_.exchange(
            msg->data);

        const bool old_clear_active =
          rm_task == clear_task_value_ ||
          old_fortress != 0;

        const bool new_clear_active =
          rm_task == clear_task_value_ ||
          msg->data != 0;

        if (old_clear_active != new_clear_active) {
          {
            std::lock_guard<
              nav2_costmap_2d::Costmap2D::mutex_t>
              lock(*getMutex());

            has_updated_data_ = true;
            current_ = false;
          }

          // RCLCPP_INFO(
          //   logger_,
          //   "rm_task=%d fortress_enable=%d: "
          //   "static clear region %s",
          //   rm_task,
          //   msg->data,
          //   new_clear_active ?
          //   "ENABLED" : "DISABLED");
        }
      });
  RCLCPP_INFO(
    logger_,
    "Static clear region initialized: "
    "x=[%.2f, %.2f], y=[%.2f, %.2f], "
    "topic=%s, trigger=%d",
    clear_min_x_,
    clear_max_x_,
    clear_min_y_,
    clear_max_y_,
    rm_task_topic_.c_str(),
    clear_task_value_);
}


void xxStaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  // 每次先正常写入/map中的静态地图
  nav2_costmap_2d::StaticLayer::updateCosts(
    master_grid,
    min_i,
    min_j,
    max_i,
    max_j);

  if (!enabled_ || !clear_region_enabled_) {
    return;
  }

  // 只有rm_task等于3时才清除静态障碍
  const bool clear_active =
    rm_task_value_.load() == clear_task_value_ ||
    fortress_enable_value_.load() != 0;

  if (!clear_active) {
    return;
  }

  const double resolution =
    master_grid.getResolution();

  const double origin_x =
    master_grid.getOriginX();

  const double origin_y =
    master_grid.getOriginY();

  const int map_size_x =
    static_cast<int>(
      master_grid.getSizeInCellsX());

  const int map_size_y =
    static_cast<int>(
      master_grid.getSizeInCellsY());

  auto clearRegion =
    [&](
      double min_x,
      double max_x,
      double min_y,
      double max_y)
    {
      if (min_x > max_x) {
        std::swap(min_x, max_x);
      }

      if (min_y > max_y) {
        std::swap(min_y, max_y);
      }

      int region_min_i =
        static_cast<int>(
          std::floor(
            (min_x - origin_x) /
            resolution));

      int region_max_i =
        static_cast<int>(
          std::ceil(
            (max_x - origin_x) /
            resolution));

      int region_min_j =
        static_cast<int>(
          std::floor(
            (min_y - origin_y) /
            resolution));

      int region_max_j =
        static_cast<int>(
          std::ceil(
            (max_y - origin_y) /
            resolution));

      region_min_i =
        std::clamp(
          region_min_i,
          0,
          map_size_x);

      region_max_i =
        std::clamp(
          region_max_i,
          0,
          map_size_x);

      region_min_j =
        std::clamp(
          region_min_j,
          0,
          map_size_y);

      region_max_j =
        std::clamp(
          region_max_j,
          0,
          map_size_y);

      const int start_i =
        std::max(
          min_i,
          region_min_i);

      const int end_i =
        std::min(
          max_i,
          region_max_i);

      const int start_j =
        std::max(
          min_j,
          region_min_j);

      const int end_j =
        std::min(
          max_j,
          region_max_j);

      for (
        int i = start_i;
        i < end_i;
        ++i)
      {
        for (
          int j = start_j;
          j < end_j;
          ++j)
        {
          master_grid.setCost(
            static_cast<unsigned int>(i),
            static_cast<unsigned int>(j),
            nav2_costmap_2d::FREE_SPACE);
        }
      }
    };
    clearRegion(
    clear_min_x_,
    clear_max_x_,
    clear_min_y_,
    clear_max_y_);
    clearRegion(
    16.3,
    19.0,
    -1.9,
    0.8);
}

}  // namespace xx_nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(
  xx_nav2_costmap_2d::xxStaticLayer,
  nav2_costmap_2d::Layer)