#pragma once

#include <atomic>
#include <string>

#include "nav2_costmap_2d/static_layer.hpp"
#include "std_msgs/msg/int32.hpp"

namespace xx_nav2_costmap_2d
{

class xxStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  xxStaticLayer() = default;
  ~xxStaticLayer() override = default;

  void onInitialize() override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

private:
  bool clear_region_enabled_{true};

  double clear_min_x_{1.5};
  double clear_max_x_{4.5};
  double clear_min_y_{-1.8};
  double clear_max_y_{0.8};

  // rm_task控制
  std::string rm_task_topic_{"/rm_task"};
  int clear_task_value_{3};
  std::atomic<int> rm_task_value_{0};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
    rm_task_sub_;
  std::string fortress_enable_topic_{"/fortress_enable"};
  std::atomic<int> fortress_enable_value_{0};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
    fortress_enable_sub_;
};

}  // namespace xx_nav2_costmap_2d