#ifndef THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
#define THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_

#include <iostream>
#include <cmath>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "theta_star_planner/theta_star.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace theta_star_planner
{

class ThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThetaStarPlanner")};
  std::string global_frame_, name_;
  bool use_final_approach_orientation_;

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;

  std::unique_ptr<theta_star::ThetaStar> planner_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;


  /**
   * @brief the function responsible for calling the algorithm and retrieving a path from it
   * @return global_path is the planned path to be taken
   */
  void getPlan(nav_msgs::msg::Path & global_path);

  /**
   * @brief interpolates points between the consecutive waypoints of the path
   * @param raw_path is used to send in the path received from the planner
   * @param dist_bw_points is used to send in the interpolation_resolution (which has been set as the costmap resolution)
   * @return the final path with waypoints at a distance of the value of interpolation_resolution of each other
   */
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // 上一次接受的路径
  nav_msgs::msg::Path last_path_;
  geometry_msgs::msg::PoseStamped last_goal_;

  bool has_last_path_{false};
  size_t last_progress_index_{0};

  double same_goal_tolerance_{0.15};
  double switch_improvement_ratio_{0.12};
  double max_reuse_path_deviation_{0.30};

  double compare_distance_weight_{8.0};
  double compare_traversal_weight_{1.0};
  bool tryReuseLastPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & reused_path);

  bool segmentCostAndSafe(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    double & cost) const;

  bool pathCostAndSafe(
    const geometry_msgs::msg::PoseStamped & start,
    const nav_msgs::msg::Path & path,
    double & cost) const;

  void cachePath(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal);
};
}   //  namespace theta_star_planner

#endif  //  THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_