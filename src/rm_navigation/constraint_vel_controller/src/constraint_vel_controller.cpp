// goal_approach_controller.cpp
// Nav2控制器wrapper：在接近目标时限制线速度，防止高速冲过目标点
// 原理：透明代理内部控制器（如MPPI），仅在距目标 < approach_distance 时
//       将合速度钳位到 approach_velocity

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <numeric>
// #include <Eigen/Dense>

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
namespace constraint_vel_controller
{

class ConstraintVelController : public nav2_core::Controller
{
public:
ConstraintVelController() = default;
  ~ConstraintVelController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    tf_ = tf;
    // 声明本wrapper的参数
    declare_parameter_if_not_declared(
      node, name + ".inner_plugin",
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));
    declare_parameter_if_not_declared(
      node, name + ".approach_distance",
      rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(
      node, name + ".control_frequency",
      rclcpp::ParameterValue(1.5));  
    declare_parameter_if_not_declared(
      node, name + ".approach_velocity",
      rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
      node, name + ".direct_approach_distance",
      rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
      node, name + ".direct_approach_kp",
      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node, name + ".lookahead_dist", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(
      node, name + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(
      node, name + ".max_lookahead_dist", rclcpp::ParameterValue(0.8));
    declare_parameter_if_not_declared(
      node, name + ".lookahead_time", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node, name + ".use_interpolation", rclcpp::ParameterValue(true));
    // declare_parameter_if_not_declared(
    //   node, name + ".max_robot_pose_search_dist",
    //   rclcpp::ParameterValue(getCostmapMaxExtent()));  
    declare_parameter_if_not_declared(
      node, name + ".curvature_min", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(
      node, name + ".curvature_max", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(
      node, name + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
      node, name + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(
      node, name + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(
      node, name + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(
      node, name + ".last_vel", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
      node, name + ".lower_speed", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
      node, name + ".min_dist", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
      node, name + ".large_slow", rclcpp::ParameterValue(1.0));
    std::string inner_plugin_type;
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);
    node->get_parameter(name + ".approach_distance", approach_distance_);
    node->get_parameter(name + ".approach_velocity", approach_velocity_);
    node->get_parameter(name + ".direct_approach_distance", direct_approach_distance_);
    node->get_parameter(name + ".direct_approach_kp", direct_approach_kp_);
    // node->get_parameter(name + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
    node->get_parameter(name + ".curvature_min", curvature_min_);
    node->get_parameter(name + ".curvature_max", curvature_max_);
    node->get_parameter(
      name + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
    node->get_parameter(name + ".curvature_forward_dist", curvature_forward_dist_);
    node->get_parameter(name + ".curvature_backward_dist", curvature_backward_dist_);
    node->get_parameter(
      name + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
    node->get_parameter("controller_frequency", control_frequency);
    node->get_parameter(name + ".last_vel", last_vel_);
    node->get_parameter(name + ".lower_speed", lower_speed_);
    node->get_parameter(name + ".large_slow", large_slow_);
    node->get_parameter(name + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(name + ".max_lookahead_dist", max_lookahead_dist_);
    node->get_parameter(name + ".lookahead_time", lookahead_time_);
    node->get_parameter(name + ".use_interpolation", use_interpolation_);
    node->get_parameter(name + ".lookahead_dist", lookahead_dist_);

    control_duration_ = 1.0 / control_frequency;
    // 通过pluginlib加载内部控制器
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);
    inner_controller_->configure(parent, name, tf, costmap_ros);

    RCLCPP_INFO(
      logger_,
      "GoalApproachController: 包装 [%s], approach_distance=%.2f m, approach_velocity=%.2f m/s, "
      "direct_approach_distance=%.2f m, direct_approach_kp=%.2f",
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_,
      direct_approach_distance_, direct_approach_kp_);
  }

  void cleanup() override
  {
    inner_controller_->cleanup();
  }

  void activate() override
  {
    inner_controller_->activate();
  }

  void deactivate() override
  {
    inner_controller_->deactivate();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
    }
    global_plan_ = path; 
    inner_controller_->setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    auto transformed_plan = transformGlobalPlan(pose);
    double lookahead_dist = getLookAheadDistance(velocity);
    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
    double dx = goal_.pose.position.x - pose.pose.position.x;
    double dy = goal_.pose.position.y - pose.pose.position.y;
    double goal_dist = std::hypot(dx,dy);
    double dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    double target_speed = 0.0;
    
    if (goal_dist < direct_approach_distance_) {

      target_speed = (goal_dist < approach_distance_) ? approach_velocity_ : (dist * direct_approach_kp_);
      double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
      if (dist > 0.03) {
        cmd.twist.linear.x = target_speed * cos(theta_dist);
        cmd.twist.linear.y = target_speed * sin(theta_dist);
      } else {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
      }
      cmd.twist.angular.z = 0.0;
    } 
    // else if (dist < approach_distance_) {
    //   double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
    //   if (speed > approach_velocity_) {
    //     double scale = approach_velocity_ / speed;
    //     cmd.twist.linear.x *= scale;
    //     cmd.twist.linear.y *= scale;
    //     // 角速度也按比例降低，避免原地打转
    //     cmd.twist.angular.z *= scale;
    //   }
    // }
    double linear_speed = 0.0;
    linear_speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
    double limited_speed = linear_speed;
    
    applyCurvatureLimitation(transformed_plan, carrot_pose, limited_speed);
    double scale_ratio = limited_speed / linear_speed;
    
    // 按比例缩放 x 和 y，保持运动方向不变
    cmd.twist.linear.x *= scale_ratio;
    cmd.twist.linear.y *= scale_ratio;
    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
  {
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance

  double lookahead_dist = lookahead_dist_;
  // if (use_velocity_scaled_lookahead_dist_) {
  //   double vel_to_use = 0.0;
  //   if (has_prev_cmd_vel_) {
  //     // 计算历史速度的合成线速度（vx, vy的模长）
  //     vel_to_use = hypot(
  //       prev_cmd_vel_.twist.linear.x,
  //       prev_cmd_vel_.twist.linear.y
  //     );
  //   } else {
  //     // 第一次运行，用当前反馈速度
  //     vel_to_use = hypot(speed.linear.x, speed.linear.y);
  //   }
  //   lookahead_dist = vel_to_use * lookahead_time_;
  //   lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  // }

  return lookahead_dist;
  }
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const
  {
    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }
  
    try {
      tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }
    return false;
  }
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
  {
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(
      transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
      });
  
    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == transformed_plan.poses.end()) {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
      // Find the point on the line segment between the two poses
      // that is exactly the lookahead distance away from the robot pose (the origin)
      // This can be found with a closed form for the intersection of a segment and a circle
      // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
      // and goal_pose is guaranteed to be outside the circle.
      auto prev_pose_it = std::prev(goal_pose_it);
      auto point = circleSegmentIntersection(
        prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = prev_pose_it->header.frame_id;
      pose.header.stamp = goal_pose_it->header.stamp;
      pose.pose.position = point;
      return pose;
    }
  
    return *goal_pose_it;
  }
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
  {
    // Formula for intersection of a line with a circle centered at the origin,
    // modified to always return the point that is on the segment between the two points.
    // https://mathworld.wolfram.com/Circle-LineIntersection.html
    // This works because the poses are transformed into the robot frame.
    // This can be derived from solving the system of equations of a line and a circle
    // which results in something that is just a reformulation of the quadratic formula.
    // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
    // https://www.desmos.com/calculator/td5cwbuocd
    double x1 = p1.x;
    double x2 = p2.x;
    double y1 = p1.y;
    double y2 = p2.y;
  
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double d = x1 * y2 - x2 * y1;
  
    // Augmentation to only return point within segment
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;
  
    geometry_msgs::msg::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - d * d);
    p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
  }
  double getCostmapMaxExtent() const
  {
    const double max_costmap_dim_meters =
      std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    return max_costmap_dim_meters / 2.0;
  }
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    if (global_plan_.poses.empty()) {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    double max_costmap_extent = getCostmapMaxExtent();

    auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
      global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

    // First find the closest pose on the path to the robot
    // bounded by when the path turns around (if it does) so we don't get a pose from a later
    // portion of the path
    auto transformation_begin = nav2_util::geometry_utils::min_by(
      global_plan_.poses.begin(), closest_pose_upper_bound,
      [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
        return euclidean_distance(robot_pose, ps);
      });

    // Find points up to max_transform_dist so we only transform them.
    auto transformation_end = std::find_if(
      transformation_begin, global_plan_.poses.end(),
      [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

    // Lambda to transform a PoseStamped from global frame to local
    auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
      transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
      transform_global_pose_to_local);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = robot_pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    // local_path_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel)
  {
    double curvature =
      calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);
    RCLCPP_DEBUG(logger_, "Curvature: %.3f", curvature);
    if(slow && curvature <= large_slow_-0.2) slow = false;
    if(!slow && curvature >= large_slow_) slow = true;
    double scaled_linear_vel = linear_vel;
    // std::cout << "cur " << curvature << std::endl;
    // std::cout << "slow " << slow << std::endl;
    
    if (curvature > curvature_min_) {
      double reduction_ratio = 1.0;
      if (curvature > curvature_max_) {
        reduction_ratio = reduction_ratio_at_high_curvature_;
      } else {
        reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                  (1.0 - reduction_ratio_at_high_curvature_);
      }
  
      double target_scaled_vel = linear_vel * reduction_ratio;
  
      if(slow && last_velocity_scaling_factor_ >= last_vel_){
        // scaled_linear_vel = last_vel_;
        scaled_linear_vel =
        last_velocity_scaling_factor_ + std::clamp(
                                          last_vel_-last_velocity_scaling_factor_,
                                          -lower_speed_ ,
                                          lower_speed_ );
      }  
      else if(!slow && last_velocity_scaling_factor_ > target_scaled_vel){
        scaled_linear_vel =
        last_velocity_scaling_factor_ + std::clamp(
                                          target_scaled_vel - last_velocity_scaling_factor_,
                                          -max_velocity_scaling_factor_rate_ * control_duration_,
                                          max_velocity_scaling_factor_rate_ * control_duration_);
      }                                    
    }
    // scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);
    RCLCPP_DEBUG(logger_, "Scaled linear vel: %.3f", scaled_linear_vel);
    linear_vel = std::min(linear_vel, scaled_linear_vel);
    // std::lock_guard<std::mutex> lock(sm_mutex);
    last_velocity_scaling_factor_ = linear_vel;
    // std::cout << "vel " << last_velocity_scaling_factor_ << std::endl;
  }
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const
  {
    geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
    std::vector<double> cumulative_distances = calculateCumulativeDistances(path);
  
    double lookahead_pose_cumulative_distance = 0.0;
    geometry_msgs::msg::PoseStamped robot_base_frame_pose;
    robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
    lookahead_pose_cumulative_distance =
      nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);
  
    backward_pose = findPoseAtDistance(
      path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);
  
    forward_pose = findPoseAtDistance(
      path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);
  
    double curvature_radius = calculateCurvatureRadius(
      backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
    double curvature = 1.0 / curvature_radius;
    // visualizeCurvaturePoints(backward_pose, forward_pose);
    return curvature;
  }
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const
  {
    double x1 = near_point.x, y1 = near_point.y;
    double x2 = current_point.x, y2 = current_point.y;
    double x3 = far_point.x, y3 = far_point.y;
  
    double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                       (x3 * x3 + y3 * y3) * (y1 - y2)) /
                      (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
    double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                       (x3 * x3 + y3 * y3) * (x2 - x1)) /
                      (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
    double radius = std::hypot(x2 - center_x, y2 - center_y);
    if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
      return 1e9;
    }
    return radius;
  }
  std::vector<double> calculateCumulativeDistances(
    const nav_msgs::msg::Path & path) const
  {
    std::vector<double> cumulative_distances;
    cumulative_distances.push_back(0.0);
  
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & prev_pose = path.poses[i - 1].pose.position;
      const auto & curr_pose = path.poses[i].pose.position;
      double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
      cumulative_distances.push_back(cumulative_distances.back() + distance);
    }
    return cumulative_distances;
  }
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const
  {
    if (path.poses.empty() || cumulative_distances.empty()) {
      return geometry_msgs::msg::PoseStamped();
    }
    if (target_distance <= 0.0) {
      return path.poses.front();
    }
    if (target_distance >= cumulative_distances.back()) {
      return path.poses.back();
    }
    auto it =
      std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
    size_t index = std::distance(cumulative_distances.begin(), it);
  
    if (index == 0) {
      return path.poses.front();
    }
  
    double ratio = (target_distance - cumulative_distances[index - 1]) /
                   (cumulative_distances[index] - cumulative_distances[index - 1]);
    geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
    geometry_msgs::msg::PoseStamped pose2 = path.poses[index];
  
    geometry_msgs::msg::PoseStamped interpolated_pose;
    interpolated_pose.header = pose2.header;
    interpolated_pose.pose.position.x =
      pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
    interpolated_pose.pose.position.y =
      pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
    interpolated_pose.pose.position.z =
      pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
    interpolated_pose.pose.orientation = pose2.pose.orientation;
  
    return interpolated_pose;
  }

private:

  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};
  geometry_msgs::msg::PoseStamped goal_;
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double direct_approach_distance_{0.5};
  double direct_approach_kp_{1.0};
  double max_robot_pose_search_dist_;
  double large_slow_;
  double last_vel_;
  double lower_speed_;
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double max_velocity_scaling_factor_rate_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  double lookahead_dist_;
  bool use_interpolation_;
  double last_velocity_scaling_factor_;
  bool has_prev_cmd_vel_ = false;
  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  tf2::Duration transform_tolerance_ = tf2::durationFromSec(0.1);
  double control_duration_, control_frequency;
  bool slow = false;
};

}  // namespace goal_approach_controller

PLUGINLIB_EXPORT_CLASS(
  constraint_vel_controller::ConstraintVelController,
  nav2_core::Controller)
