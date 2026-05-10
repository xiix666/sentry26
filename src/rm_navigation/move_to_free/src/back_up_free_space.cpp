#include "back_up_free_space.hpp"
#include <limits>
using ServiceResponseFuture = rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture;
namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("/global_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  // auto result = costmap_client_->async_send_request(request);
  // if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
  //   RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
  //   return nav2_behaviors::Status::FAILED;
  // }

  // // get costmap
  // auto costmap = result.get()->map;
  // using ServiceResponseFuture = rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture;
  auto response_callback = [this](ServiceResponseFuture future) {
    try {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      latest_costmap_ = future.get()->map;
      has_costmap_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger_, "GetCostmap failed: %s", e.what());
    }
  };
  costmap_client_->async_send_request(request, response_callback);
  nav2_msgs::msg::Costmap costmap_copy;

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!has_costmap_) {
      RCLCPP_WARN(logger_, "No costmap received yet");
      return nav2_behaviors::Status::RUNNING;
    }
    costmap_copy = latest_costmap_;
  }

  // if (!nav2_util::getCurrentPose(
  //       initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
  //   RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
  //   return nav2_behaviors::Status::FAILED;
  // }
  geometry_msgs::msg::TransformStamped tf_stamped;
  geometry_msgs::msg::Pose2D pose;
  try {
    tf_stamped = tf_->lookupTransform(
      global_frame_,          // target frame
      robot_base_frame_,      // source frame
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  init_x = tf_stamped.transform.translation.x;
  init_y = tf_stamped.transform.translation.y;
  
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Initial robot pose is not available: %s",
      ex.what());
    return nav2_behaviors::Status::FAILED;
  }
  parseCostmapAndQuery(costmap_copy);
  // float min_avg_cost = 255.0f;
  // float best_direction_rad = 0.0f;

  int min_leading_obstacle_count = std::numeric_limits<int>::max();
  float min_avg_cost = 256.0f;
  float best_direction_rad = 0.0f;
  bool has_safe_direction = false;
  
  for (int i = 0; i < self_save_sample_directions_; ++i) {
    float direction_rad =
      2.0f * static_cast<float>(M_PI) * i / self_save_sample_directions_;
  
    std::vector<float> cost_list =
      getCostListInDirection(costmap_copy, init_x, init_y, direction_rad);
  
    if (cost_list.empty()) {
      continue;
    }
  
    int leading_obstacle_count = countLeadingObstacleCount(cost_list);
    float avg_cost = calculateAvgCost(cost_list);
  
    if (!has_safe_direction ||
        leading_obstacle_count < min_leading_obstacle_count ||
        (leading_obstacle_count == min_leading_obstacle_count && avg_cost < min_avg_cost))
    {
      min_leading_obstacle_count = leading_obstacle_count;
      min_avg_cost = avg_cost;
      best_direction_rad = direction_rad;
      has_safe_direction = true;
    }
  }
  
  if (!has_safe_direction) {
    RCLCPP_WARN(logger_, "BackUpFreeSpace failed to find any sampled direction.");
    return nav2_behaviors::Status::FAILED;
  }
  
  RCLCPP_WARN(
    logger_,
    "BackUpFreeSpace best direction: %.2f rad, leading_obstacles=%d, avg_cost=%.2f",
    best_direction_rad,
    min_leading_obstacle_count,
    min_avg_cost);

  // Calculate move command
  twist_x_ = std::cos(best_direction_rad) * command->speed;
  twist_y_ = std::sin(best_direction_rad) * command->speed;
  command_x_ = command->target.x;
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;

  // if (!nav2_util::getCurrentPose(
  //       initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
  //   RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
  //   return nav2_behaviors::Status::FAILED;
  // }
  RCLCPP_WARN(
    logger_, "backing up %f meters towards free space at angle %f", command_x_, best_direction_rad);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }
  geometry_msgs::msg::TransformStamped tf_stamped;
  geometry_msgs::msg::Pose2D pose;
  try {
    tf_stamped = tf_->lookupTransform(
      global_frame_,          // target frame
      robot_base_frame_,      // source frame
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  current_x = tf_stamped.transform.translation.x;
  current_y = tf_stamped.transform.translation.y;
  
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Initial robot pose is not available: %s",
      ex.what());
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = init_x - current_x;
  float diff_y = init_y - current_y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }
  requestCostmapAsync();
  nav2_msgs::msg::Costmap costmap_copy;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);

    if (!has_costmap_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "No costmap received yet, waiting...");
      return nav2_behaviors::Status::RUNNING;
    }

    costmap_copy = latest_costmap_;
  }

  // auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  // auto result = costmap_client_->async_send_request(request);
  // if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
  //   RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
  //   return nav2_behaviors::Status::FAILED;
  // }

  // auto costmap = result.get()->map;

  int cost = parseCostmapAndQuery(costmap_copy);
  if (distance >= std::fabs(command_x_) || (cost >= 0 && cost <= 150)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }
  if (!has_free_space_direction_) {
    int min_leading_obstacle_count = std::numeric_limits<int>::max();
    float min_avg_cost = 256.0f;
    float best_direction_rad = 0.0f;
    bool has_safe_direction = false;

    for (int i = 0; i < self_save_sample_directions_; ++i) {
      float direction_rad =
        2.0f * static_cast<float>(M_PI) * i / self_save_sample_directions_;

      std::vector<float> cost_list =
        getCostListInDirection(costmap_copy, init_x, init_y, direction_rad);

      if (cost_list.empty()) {
        continue;
      }

      int leading_obstacle_count = countLeadingObstacleCount(cost_list);
      float avg_cost = calculateAvgCost(cost_list);

      if (!has_safe_direction ||
          leading_obstacle_count < min_leading_obstacle_count ||
          (leading_obstacle_count == min_leading_obstacle_count &&
           avg_cost < min_avg_cost))
      {
        min_leading_obstacle_count = leading_obstacle_count;
        min_avg_cost = avg_cost;
        best_direction_rad = direction_rad;
        has_safe_direction = true;
      }
    }

    if (!has_safe_direction) {
      RCLCPP_WARN(
        logger_,
        "BackUpFreeSpace failed to find any sampled direction.");
      return nav2_behaviors::Status::FAILED;
    }

    twist_x_ = std::cos(best_direction_rad) * std::fabs(command_x_ >= 0.0 ? twist_x_ : twist_x_);
    twist_y_ = std::sin(best_direction_rad) * std::fabs(twist_y_);
    has_free_space_direction_ = true;

    RCLCPP_WARN(
      logger_,
      "BackUpFreeSpace best direction: %.2f rad, leading_obstacles=%d, avg_cost=%.2f",
      best_direction_rad,
      min_leading_obstacle_count,
      min_avg_cost);
  }
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}
void BackUpFreeSpace::requestCostmapAsync()
{
  if (!costmap_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "GetCostmap service is not ready.");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (costmap_request_in_flight_) {
      return;
    }
    costmap_request_in_flight_ = true;
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();

  auto response_callback = [this](ServiceResponseFuture future) {
    try {
      auto response = future.get();

      std::lock_guard<std::mutex> lock(costmap_mutex_);
      latest_costmap_ = response->map;
      has_costmap_ = true;
      costmap_request_in_flight_ = false;
    } catch (const std::exception & e) {
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap_request_in_flight_ = false;
      }

      RCLCPP_ERROR(
        logger_,
        "GetCostmap failed: %s", e.what());
    }
  };

  costmap_client_->async_send_request(request, response_callback);
}
int BackUpFreeSpace::parseCostmapAndQuery(const nav2_msgs::msg::Costmap& costmap) {
  const auto& metadata = costmap.metadata;
  float resolution = metadata.resolution;         
  float origin_x = metadata.origin.position.x;    
  float origin_y = metadata.origin.position.y;    
  int width = metadata.size_x;                     
  int height = metadata.size_y;                     

  int grid_x = static_cast<int>((current_x - origin_x) / resolution);
  int grid_y = static_cast<int>((current_y - origin_y) / resolution);

  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
    RCLCPP_WARN(logger_, 
      "========== 查询结果 ==========\n"
      "目标点 (%.2f, %.2f) 超出代价地图范围！\n"
      "栅格坐标: (%d, %d)，地图尺寸: %dx%d栅格\n"
      "地图范围: X[%.2f, %.2f], Y[%.2f, %.2f]\n"
      "==================================",
      current_x, current_y, grid_x, grid_y, width, height,
      origin_x, origin_x + width*resolution,
      origin_y, origin_y + height*resolution);
    return 256;
  }

  size_t index = static_cast<size_t>(grid_y) * static_cast<size_t>(width) + static_cast<size_t>(grid_x);
  if (index >= costmap.data.size()) {
    RCLCPP_ERROR(logger_, "无效栅格索引: %zu（总长度：%zu）", 
      index, costmap.data.size());
    return 256;
  }

  unsigned char cost = costmap.data[index];
  int c = cost;
  return c;

}
int BackUpFreeSpace::countLeadingObstacleCount(const std::vector<float>& cost_list) const
{
  const float OBSTACLE_THRESHOLD = 150.0f;

  int count = 0;
  for (float cost : cost_list) {
    if (cost >= OBSTACLE_THRESHOLD) {
      count++;
    } else {
      break;
    }
  }

  return count;
}
void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
