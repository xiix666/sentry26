#ifndef PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief An enhanced back_up action that move toward free space
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  std::vector<float> getCostListInDirection(const nav2_msgs::msg::Costmap& costmap,
    float robot_x_map, float robot_y_map, float direction_rad) const
  {
    std::vector<float> cost_list;
    const auto& meta = costmap.metadata;
    float resolution = meta.resolution;
    float origin_x = meta.origin.position.x;
    float origin_y = meta.origin.position.y;
    int width = meta.size_x;
    int height = meta.size_y;
    float sample_step = resolution;
    int total_sample_points = static_cast<int>(self_save_sample_radius_ / sample_step);
    if (total_sample_points < 1) total_sample_points = 1;
  
    for (int step = 1; step <= total_sample_points; ++step) {
        float sample_x = robot_x_map + step * sample_step * std::cos(direction_rad);
        float sample_y = robot_y_map + step * sample_step * std::sin(direction_rad);
        int grid_x = static_cast<int>((sample_x - origin_x) / resolution);
        int grid_y = static_cast<int>((sample_y - origin_y) / resolution);
        size_t idx = static_cast<size_t>(grid_y) * static_cast<size_t>(width) + static_cast<size_t>(grid_x);
        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height || idx >= costmap.data.size()) {
            cost_list.push_back(255.0f);
        } else {
            cost_list.push_back(static_cast<float>(costmap.data[idx]));
        }
    }
    return cost_list;
  }
  
  bool isDirectionSafe(const std::vector<float>& cost_list) const
  {
    const int REQUIRED_DECREASE_POINTS = 3;
    const float COST_THRESHOLD = 150.0f;
    if (cost_list.size() <= static_cast<size_t>(REQUIRED_DECREASE_POINTS)) return false;
    for (size_t i = 0; i + REQUIRED_DECREASE_POINTS < cost_list.size(); ++i) {
        if (cost_list[i] >= COST_THRESHOLD) continue;
        bool is_decreasing = true;
        for (int j = 0; j < REQUIRED_DECREASE_POINTS; ++j) {
            if (cost_list[i + j + 1] > cost_list[i + j]) {
                is_decreasing = false;
                break;
            }
        }
        if (is_decreasing) return true;
    }
    return false;
  }
  
  float calculateAvgCost(const std::vector<float>& cost_list) const
  {
    if (cost_list.empty()) return 255.0f;
    float sum = 0.0f;
    for (float c : cost_list) sum += c;
    return sum / cost_list.size();
  }
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Cleanup server on lifecycle transition
   */
  void onCleanup() override;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  /**
   * @brief Gather free points within a specified radius from the center in the costmap.
   *
   * This function iterates through the costmap and collects points that are free (costmap value is 0)
   * and within the specified radius from the given center coordinates (center_x, center_y).
   *
   * @param costmap The costmap to search for free points.
   * @param center_x The x-coordinate of the center point.
   * @param center_y The y-coordinate of the center point.
   * @param radius The radius within which to gather free points.
   * @return A vector of points that are free and within the specified radius.
   */
  std::vector<geometry_msgs::msg::Point> gatherFreePoints(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius);
  int parseCostmapAndQuery(const nav2_msgs::msg::Costmap& costmap) ;
  void requestCostmapAsync();
  float findBestDirection(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
    float end_angle, float radius, float angle_increment);
    int countLeadingObstacleCount(const std::vector<float>& cost_list) const;
  void visualize(
    geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle);

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    marker_pub_;
  double twist_x_, twist_y_;
  double init_x,init_y;
  double current_x, current_y;
  // parameters
  std::string service_name_;
  double max_radius_;
  bool visualize_;
  double self_save_sample_radius_ = 3.0;
  int self_save_sample_directions_ = 18;
  nav2_msgs::msg::Costmap latest_costmap_;
  bool has_costmap_ = false;
  bool costmap_request_in_flight_{false};

  bool has_free_space_direction_{false};
  std::mutex costmap_mutex_;
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
