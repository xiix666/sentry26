#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "theta_star_planner/theta_star_planner.hpp"
#include "theta_star_planner/theta_star.hpp"

namespace theta_star_planner
{
void ThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<theta_star::ThetaStar>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".same_goal_tolerance",
    rclcpp::ParameterValue(0.15));
  node->get_parameter(
    name_ + ".same_goal_tolerance",
    same_goal_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".switch_improvement_ratio",
    rclcpp::ParameterValue(0.12));
  node->get_parameter(
    name_ + ".switch_improvement_ratio",
    switch_improvement_ratio_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_reuse_path_deviation",
    rclcpp::ParameterValue(0.50));
  node->get_parameter(
    name_ + ".max_reuse_path_deviation",
    max_reuse_path_deviation_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".compare_distance_weight",
    rclcpp::ParameterValue(8.0));
  node->get_parameter(
    name_ + ".compare_distance_weight",
    compare_distance_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".compare_traversal_weight",
    rclcpp::ParameterValue(1.0));
  node->get_parameter(
    name_ + ".compare_traversal_weight",
    compare_traversal_weight_);

  same_goal_tolerance_ = std::max(0.0, same_goal_tolerance_);
  switch_improvement_ratio_ =
    std::clamp(switch_improvement_ratio_, 0.0, 0.99);
  max_reuse_path_deviation_ =
    std::max(0.0, max_reuse_path_deviation_);
}

void ThetaStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type theta_star_planner", name_.c_str());
  has_last_path_ = false;
  last_path_.poses.clear();
  last_progress_index_ = 0;
  planner_.reset();
}

void ThetaStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type theta_star_planner", name_.c_str());
  auto node = parent_node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ThetaStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void ThetaStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type theta_star_planner", name_.c_str());
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));

  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    RCLCPP_WARN(logger_, "Start Coordinates were outside map bounds");
    return global_path;
  }

  if (!planner_->costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    RCLCPP_WARN(logger_, "Goal Coordinates were outside map bounds");
    return global_path;
  }

  if (mx_start == mx_goal && my_start == my_goal) {
    if (planner_->costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
      return global_path;
    }
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;

    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  nav_msgs::msg::Path new_path;
  planner_->setStartAndGoal(start, goal);

  RCLCPP_DEBUG(
    logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);

  getPlan(new_path);

  nav_msgs::msg::Path reused_path;
  const bool reused_last_path =
    tryReuseLastPath(start, goal, new_path, reused_path);

  if (reused_last_path) {
    global_path = std::move(reused_path);
    RCLCPP_DEBUG(
      logger_, "Publishing pruned reused path with %zu poses",
      global_path.poses.size());
  } else {
    global_path = std::move(new_path);
  }

  size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  if (use_final_approach_orientation_) {
    if (plan_size == 1) {
      global_path.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = global_path.poses.back().pose.position;
      auto approach_pose = global_path.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      global_path.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }

  if (!reused_last_path && !global_path.poses.empty()) {
    cachePath(global_path, goal);
  }
  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  return global_path;
}

void ThetaStarPlanner::getPlan(nav_msgs::msg::Path & global_path)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    RCLCPP_WARN(
      logger_,
      "Start or goal is in obstacle, trying to move it to nearest free cell..."
    );

    if (!planner_->moveStartAndGoalToNearestFreeCell(20)) {
      RCLCPP_ERROR(
        logger_,
        "Could not find free cell around start or goal pose"
      );
      global_path.poses.clear();
      global_path.header.stamp = clock_->now();
      global_path.header.frame_id = global_frame_;
      return;
    }
  }
  if (planner_->generatePath(path)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    global_path.poses.clear();
  }
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;

  for (auto & pose : global_path.poses) {
    pose.header = global_path.header;

    if (
      pose.pose.orientation.x == 0.0 &&
      pose.pose.orientation.y == 0.0 &&
      pose.pose.orientation.z == 0.0 &&
      pose.pose.orientation.w == 0.0)
    {
      pose.pose.orientation.w = 1.0;
    }
  }
}
bool ThetaStarPlanner::segmentCostAndSafe(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  double & cost) const
{
  cost = 0.0;

  const double dx =
    to.pose.position.x - from.pose.position.x;
  const double dy =
    to.pose.position.y - from.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double resolution = planner_->costmap_->getResolution();

  cost += compare_distance_weight_ * distance / resolution;

  const int steps = std::max(
    1,
    static_cast<int>(
      std::ceil(distance / (0.5 * resolution))));

  for (int i = 0; i <= steps; ++i) {
    const double ratio =
      static_cast<double>(i) / static_cast<double>(steps);

    const double wx =
      from.pose.position.x + ratio * dx;
    const double wy =
      from.pose.position.y + ratio * dy;

    unsigned int mx;
    unsigned int my;

    if (!planner_->costmap_->worldToMap(wx, wy, mx, my)) {
      return false;
    }

    if (!planner_->isSafe(
        static_cast<int>(mx),
        static_cast<int>(my)))
    {
      return false;
    }

    const unsigned char raw_cost =
      planner_->costmap_->getCost(mx, my);

    double scaled_cost;
    if (raw_cost == UNKNOWN_COST) {
      scaled_cost = OCCUPIED_COST - 1;
    } else {
      scaled_cost = 26.0 + 0.9 * raw_cost;
    }

  cost +=
    compare_traversal_weight_ *
    scaled_cost * scaled_cost /
    MAX_NON_OBSTACLE_COST /
    MAX_NON_OBSTACLE_COST;
  }

  return true;
}
bool ThetaStarPlanner::pathCostAndSafe(
  const geometry_msgs::msg::PoseStamped & start,
  const nav_msgs::msg::Path & path,
  double & cost) const
{
  cost = 0.0;

  if (path.poses.empty()) {
    return false;
  }

  double segment_cost = 0.0;
  if (!segmentCostAndSafe(start, path.poses.front(), segment_cost)) {
    return false;
  }
  cost += segment_cost;

  for (size_t i = 1; i < path.poses.size(); ++i) {
    if (!segmentCostAndSafe(path.poses[i - 1], path.poses[i], segment_cost)) {
      return false;
    }
    cost += segment_cost;
  }

  return true;
}

void ThetaStarPlanner::cachePath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal)
{
  last_path_ = path;
  last_goal_ = goal;
  last_progress_index_ = 0;
  has_last_path_ = last_path_.poses.size() >= 2;
}

bool ThetaStarPlanner::tryReuseLastPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & reused_path)
{
  if (!has_last_path_ || last_path_.poses.size() < 2) {
    return false;
  }

  const double goal_distance = std::hypot(
    goal.pose.position.x - last_goal_.pose.position.x,
    goal.pose.position.y - last_goal_.pose.position.y);

  if (goal_distance > same_goal_tolerance_) {
    return false;
  }

  const size_t search_begin = std::min(
    last_progress_index_,
    last_path_.poses.size() - 2);

  size_t nearest_segment = search_begin;
  double nearest_ratio = 0.0;
  double nearest_distance_sq =
    std::numeric_limits<double>::max();

  for (size_t i = search_begin;
    i + 1 < last_path_.poses.size(); ++i)
  {
    const auto & p0 = last_path_.poses[i].pose.position;
    const auto & p1 = last_path_.poses[i + 1].pose.position;

    const double segment_x = p1.x - p0.x;
    const double segment_y = p1.y - p0.y;
    const double segment_length_sq =
      segment_x * segment_x + segment_y * segment_y;

    double ratio = 0.0;
    if (segment_length_sq > 1e-12) {
      ratio =
        ((start.pose.position.x - p0.x) * segment_x +
        (start.pose.position.y - p0.y) * segment_y) /
        segment_length_sq;
      ratio = std::clamp(ratio, 0.0, 1.0);
    }

    const double projected_x = p0.x + ratio * segment_x;
    const double projected_y = p0.y + ratio * segment_y;
    const double dx = start.pose.position.x - projected_x;
    const double dy = start.pose.position.y - projected_y;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_segment = i;
      nearest_ratio = ratio;
    }
  }

  const double path_deviation = std::sqrt(nearest_distance_sq);
  if (path_deviation > max_reuse_path_deviation_) {
    RCLCPP_INFO(
      logger_,
      "Reject old path: robot deviation %.3f m exceeds %.3f m",
      path_deviation, max_reuse_path_deviation_);
    return false;
  }

  reused_path.poses.clear();
  reused_path.header.stamp = clock_->now();
  reused_path.header.frame_id = global_frame_;

  auto projection_pose = last_path_.poses[nearest_segment];
  const auto & segment_start =
    last_path_.poses[nearest_segment].pose.position;
  const auto & segment_end =
    last_path_.poses[nearest_segment + 1].pose.position;
  projection_pose.header = reused_path.header;
  projection_pose.pose.position.x =
    segment_start.x + nearest_ratio * (segment_end.x - segment_start.x);
  projection_pose.pose.position.y =
    segment_start.y + nearest_ratio * (segment_end.y - segment_start.y);
  projection_pose.pose.position.z =
    segment_start.z + nearest_ratio * (segment_end.z - segment_start.z);
  reused_path.poses.push_back(projection_pose);

  for (size_t i = nearest_segment + 1;
    i < last_path_.poses.size(); ++i)
  {
    auto pose = last_path_.poses[i];
    pose.header = reused_path.header;

    const auto & previous = reused_path.poses.back().pose.position;
    const double dx = pose.pose.position.x - previous.x;
    const double dy = pose.pose.position.y - previous.y;
    if (dx * dx + dy * dy < 1e-12) {
      continue;
    }
    reused_path.poses.push_back(pose);
  }

  if (reused_path.poses.size() < 2) {
    return false;
  }

  double old_cost = 0.0;
  if (!pathCostAndSafe(start, reused_path, old_cost)) {
    RCLCPP_INFO(
      logger_,
      "Reject old path: remaining path or connection is blocked");
    return false;
  }

  double new_cost = 0.0;
  const bool new_path_valid =
    pathCostAndSafe(start, new_path, new_cost);

  if (new_path_valid &&
    new_cost <= old_cost * (1.0 - switch_improvement_ratio_))
  {
    RCLCPP_DEBUG(
      logger_,
      "Use new Theta* path: new_cost=%.3f old_cost=%.3f improvement=%.1f%%",
      new_cost, old_cost,
      old_cost > 1e-9 ? (old_cost - new_cost) * 100.0 / old_cost : 0.0);
    return false;
  }

  last_progress_index_ = nearest_segment;

  RCLCPP_DEBUG(
    logger_,
    "Reuse pruned old path: progress=%zu poses=%zu deviation=%.3f "
    "old_cost=%.3f new_cost=%.3f new_valid=%d",
    nearest_segment, reused_path.poses.size(), path_deviation,
    old_cost, new_cost, new_path_valid);

  return true;
}

nav_msgs::msg::Path ThetaStarPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  geometry_msgs::msg::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }
  if (!raw_path.empty()) {
    p1.pose.position.x = raw_path.back().x;
    p1.pose.position.y = raw_path.back().y;
    p1.pose.position.z = 0.0;
    p1.pose.orientation.w = 1.0;
    pa.poses.push_back(p1);
  }
  return pa;
}

rcl_interfaces::msg::SetParametersResult
ThetaStarPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".how_many_corners") {
        planner_->how_many_corners_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      } else if (name == name_ + ".same_goal_tolerance") {
        same_goal_tolerance_ =
          std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".switch_improvement_ratio") {
        switch_improvement_ratio_ =
          std::clamp(parameter.as_double(), 0.0, 0.99);
      } else if (name == name_ + ".max_reuse_path_deviation") {
        max_reuse_path_deviation_ =
          std::max(0.0, parameter.as_double());
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        planner_->allow_unknown_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
