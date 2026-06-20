// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
// 新增：SG滤波所需头文件

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace pb_omni_pid_pursuit_controller
{

OmniPidPursuitController::OmniPidPursuitController()
  : impl_(std::make_unique<Impl>()),
    use_mpc_control_(false)  // 默认使用PID
{
  // 初始化SG滤波器
  impl_->linear_vel_filter_ = std::make_unique<SavitzkyGolayFilter>(
    impl_->sg_window_size_, impl_->sg_poly_order_);
  impl_->angular_vel_filter_ = std::make_unique<SavitzkyGolayFilter>(
    impl_->sg_window_size_, impl_->sg_poly_order_);
}


void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();
  // smoothed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 1);

  // ========== 核心改动：添加算法选择参数 ==========
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_mpc_control", rclcpp::ParameterValue(false));
  // ========== 新增：声明SG滤波相关参数 ==========
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_sg_filter", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sg_window_size", rclcpp::ParameterValue(5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sg_poly_order", rclcpp::ParameterValue(2));

  // ========== MPC参数：无论是否启用都声明（方便动态切换） ==========
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Np", rclcpp::ParameterValue(5)); 
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Nc", rclcpp::ParameterValue(3));  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Q_x", rclcpp::ParameterValue(15.0)); 
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Q_y", rclcpp::ParameterValue(15.0));  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_R_vx", rclcpp::ParameterValue(0.5)); 
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_R_vy", rclcpp::ParameterValue(0.5));  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Rdelta_vx", rclcpp::ParameterValue(0.5)); 
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Rdelta_vy", rclcpp::ParameterValue(0.5));  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".acc_max", rclcpp::ParameterValue(5.5));  
  

  // 公共参数声明（PID/MPC均需使用）
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".last_vel", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lower_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_dist", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".large_slow", rclcpp::ParameterValue(1.0));
  // 公共参数读取（PID/MPC均需使用）
  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
  node->get_parameter("controller_frequency", control_frequency);
  node->get_parameter(plugin_name_ + ".last_vel", last_vel_);
  node->get_parameter(plugin_name_ + ".lower_speed", lower_speed_);
  node->get_parameter(plugin_name_ + ".large_slow", large_slow_);
  // SG滤波参数读取
  node->get_parameter(plugin_name_ + ".enable_sg_filter", impl_->enable_sg_filter_);
  node->get_parameter(plugin_name_ + ".sg_window_size", impl_->sg_window_size_);
  node->get_parameter(plugin_name_ + ".sg_poly_order", impl_->sg_poly_order_);

  node->get_parameter(plugin_name_ + ".use_mpc_control", use_mpc_control_);

  // MPC参数读取（无论是否启用都读取，方便动态切换）
  node->get_parameter(plugin_name_ + ".mpc_Q_x", mpc_Q_x);
  node->get_parameter(plugin_name_ + ".mpc_Q_y", mpc_Q_y);
  node->get_parameter(plugin_name_ + ".mpc_R_vx", mpc_R_vx);
  node->get_parameter(plugin_name_ + ".mpc_R_vy", mpc_R_vy);
  node->get_parameter(plugin_name_ + ".mpc_Rdelta_vx", mpc_Rdelta_vx);
  node->get_parameter(plugin_name_ + ".mpc_Rdelta_vy", mpc_Rdelta_vy);
  node->get_parameter(plugin_name_ + ".mpc_Np", mpc_Np_);
  node->get_parameter(plugin_name_ + ".mpc_Nc", mpc_Nc_);
  node->get_parameter(plugin_name_ + ".min_dist", min_dist_);
  node->get_parameter(plugin_name_ + ".acc_max", acc_max_);

  control_duration_ = 1.0 / control_frequency;
  // minco_tracker_ = std::make_unique<minco_nav2::MincoTracker>();
  // minco_tracker_->setParams(v_linear_max_,acc_max_,control_duration_);
  // ========== 核心改动：根据参数初始化对应控制器 ==========
  if (use_mpc_control_) {
    // MPC模式：初始化MPC控制器
    mpc_controller_ = std::make_unique<OmniMpcController>(
        control_duration_,  // 控制周期Ts
        mpc_Np_,            // 预测时域
        mpc_Nc_,            // 控制时域
        v_linear_min_,      // 线速度最小值
        v_linear_max_       // 线速度最大值
    );
    Eigen::Matrix2d Q, R, R_delta;
    Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
    R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
    R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
    mpc_controller_->initWeights(Q, R, R_delta);
  } else {
    // PID模式：初始化PID控制器
    move_pid_ = std::make_shared<PID>(
      control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
      translation_ki_);
    heading_pid_ = std::make_shared<PID>(
      control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
    // 更新滤波器参数
    impl_->linear_vel_filter_->setWindowSize(impl_->sg_window_size_);
    impl_->linear_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
    impl_->angular_vel_filter_->setWindowSize(impl_->sg_window_size_);
    impl_->angular_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
  }

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  
  smooth_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_nav2_result",
    rclcpp::SensorDataQoS(),
    std::bind(&OmniPidPursuitController::smoothedVelCallback, this, std::placeholders::_1));
  rm_task_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/rm_task",
    10,
  [this](const std_msgs::msg::Int32::SharedPtr msg)
  {
    rm_task_value_.store(msg->data);
  });
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()
      ->create_publisher<visualization_msgs::msg::MarkerArray>(
        "curvature_points_marker_array", rclcpp::QoS(10));

  // 初始化旋转PID（PID/MPC模式均需）
  if (!use_mpc_control_) {
    heading_pid_ = std::make_shared<PID>(
      control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
  }
}

void OmniPidPursuitController::smoothedVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 加锁保护共享变量，避免线程冲突
  std::lock_guard<std::mutex> lock(sm_mutex);
  // 提取平滑后的线速度（x方向，全向机器人核心速度）
  double x = msg->linear.x;
  double y = msg->linear.y;
  smoothed_vel = std::sqrt(x*x+y*y);
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
  
  // 释放控制器资源
  if (use_mpc_control_) {
    mpc_controller_.reset();
  } else {
    move_pid_.reset();
    heading_pid_.reset();
  }
}

void OmniPidPursuitController::activate()
{
  // 核心改动：根据参数打印模式信息
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "pb_omni_pid_pursuit_controller::OmniPidPursuitController (%s Mode)",
    plugin_name_.c_str(),
    use_mpc_control_ ? "MPC" : "PID");
  
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  // 核心改动：根据参数打印模式信息
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "pb_omni_pid_pursuit_controller::OmniPidPursuitController (%s Mode)",
    plugin_name_.c_str(),
    use_mpc_control_ ? "MPC" : "PID");
  
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  double lin_dist = 0.0, theta_dist = 0.0, angle_to_goal = 0.0;

  std::vector<geometry_msgs::msg::PoseStamped> valid_path_poses;
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  // std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  // std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(velocity);

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

  bool carrot_valid = std::isfinite(carrot_pose.pose.position.x) && 
                      std::isfinite(carrot_pose.pose.position.y);
  if (!carrot_valid) {
    RCLCPP_WARN(logger_, "Carrot pose invalid (NaN/Inf), return zero vel");
    return cmd_vel;
  }

  if (!use_mpc_control_) {  
    carrot_pub_->publish (createCarrotMsg(carrot_pose));
  } else {

    for (const auto &p : transformed_plan.poses) {

      double dist_x = p.pose.position.x;
      double dist_y = p.pose.position.y;
      double dist = hypot(dist_x, dist_y);

      if (dist >= min_dist_ && dist <= lookahead_dist_) {
        valid_path_poses.push_back(p);
      }
    }
    if(valid_path_poses.empty()) {
      for (const auto &p : transformed_plan.poses) {

        double dist_x = p.pose.position.x;
        double dist_y = p.pose.position.y;
        double dist = hypot(dist_x, dist_y);
  
        if (dist > 0 && dist <= lookahead_dist_) {
          valid_path_poses.push_back(p);
        }
      }
    }
    if(!valid_path_poses.empty()) {
      lin_dist = hypot(valid_path_poses[0].pose.position.x, valid_path_poses[0].pose.position.y);
    }

  }

  if (!use_mpc_control_) {
    lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);

    if (use_rotate_to_heading_) {
      if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
        lin_dist = 0;  
      }
    }
  }

  double cmd_vx = 0.0, cmd_vy = 0.0;
  double lin_vel = 0.0;
  double angular_vel = 0.0;  
  const bool direct_drive_mode = (rm_task_value_.load() == 1);

  if (direct_drive_mode) {
    const auto & target_pose = transformed_plan.poses.back();

    const double target_x = target_pose.pose.position.x;
    const double target_y = target_pose.pose.position.y;
    const double lin_dist = std::hypot(target_x, target_y);

    if (!std::isfinite(target_x) || !std::isfinite(target_y)) {
      RCLCPP_WARN(logger_, "Direct drive target invalid, return zero vel");
      return cmd_vel;
    }

    if (!isDirectPathSafeToTarget(target_pose)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Direct drive blocked: path to target has cost > 200");
      return cmd_vel;
    }

    if (lin_dist > 1e-6) {
      double lin_vel = translation_kp_ * lin_dist;

      // 和 PID 控制一样，使用 v_linear_max_ 限制最大线速度
      lin_vel = std::clamp(lin_vel, 0.0, v_linear_max_);

      const double theta_dist = std::atan2(target_y, target_x);

      cmd_vx = lin_vel * std::cos(theta_dist);
      cmd_vy = lin_vel * std::sin(theta_dist);
    }

    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

  }
  else if (use_mpc_control_) {

    Eigen::Vector2d u_opt(0.0, 0.0);
    if (lin_dist > 0 && !valid_path_poses.empty()) {
      std::vector<Eigen::Vector2d> mpc_ref_seq = samplePathToRefSeq(valid_path_poses, mpc_Np_);
      for(int i=0;i<mpc_ref_seq.size();i++){
        std::cout << mpc_ref_seq[i](0) << " " << mpc_ref_seq[i](1) << std::endl;
      }
      Eigen::Vector2d curr_x(0.0, 0.0);
      mpc_controller_->setVelocityLimits(v_linear_min_, v_linear_max_);
      u_opt = mpc_controller_->solve(curr_x, mpc_ref_seq);
      cmd_vx = u_opt(0);
      cmd_vy = u_opt(1);
    }

    if (lin_dist > 0) {
      applyCurvatureLimitation_mpc(transformed_plan, carrot_pose, cmd_vx, cmd_vy);
      applyApproachVelocityScaling_mpc(transformed_plan, cmd_vx, cmd_vy);
    }

    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

  } else {
    lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);

    if (use_rotate_to_heading_) {
      if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
        lin_dist = 0;  
      }
    }

    if (lin_dist > 0) {
      lin_vel = move_pid_->calculate(lin_dist, 0);
      if (!std::isfinite(lin_vel)) {
        RCLCPP_WARN(logger_, "PID returned NaN/Inf, using zero vel");
        lin_vel = 0.0;
      }
      applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
      applyApproachVelocityScaling(transformed_plan, lin_vel);
      cmd_vx = lin_vel * cos(theta_dist);
      cmd_vy = lin_vel * sin(theta_dist);
      RCLCPP_DEBUG(logger_, "PID output: lin_vel=%.3f, vx=%.3f, vy=%.3f", lin_vel, cmd_vx, cmd_vy);
    }

    if (impl_->enable_sg_filter_ && lin_dist > 0) {
      double filtered_lin_vel = impl_->linear_vel_filter_->filter(lin_vel);
      cmd_vx = filtered_lin_vel * cos(theta_dist);
      cmd_vy = filtered_lin_vel * sin(theta_dist);
      RCLCPP_DEBUG(logger_, "SG filtered vel: %.3f", filtered_lin_vel);
    }

    angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;
  }
  if (!std::isfinite(cmd_vx)) cmd_vx = 0.0;
  if (!std::isfinite(cmd_vy)) cmd_vy = 0.0;
  if (!std::isfinite(angular_vel)) angular_vel = 0.0;
    if (has_prev_cmd_vel_) {
    const double max_delta_v = acc_max_ * control_duration_;
  
    const double prev_vx = prev_cmd_vel_.twist.linear.x;
    const double prev_vy = prev_cmd_vel_.twist.linear.y;
  
    const double dvx = cmd_vx - prev_vx;
    const double dvy = cmd_vy - prev_vy;
    const double delta_v_norm = std::hypot(dvx, dvy);
  
    if (delta_v_norm > max_delta_v && delta_v_norm > 1e-6) {
      const double scale = max_delta_v / delta_v_norm;
      cmd_vx = prev_vx + dvx * scale;
      cmd_vy = prev_vy + dvy * scale;
    }
  }
  double max_linear_vel = 4.0;   // 防止出意外再限制一下最大线速度
  double speed_mag = std::hypot(cmd_vx, cmd_vy);
  if (speed_mag > max_linear_vel) {
    double scale = max_linear_vel / speed_mag;
    cmd_vx *= scale;
    cmd_vy *= scale;
  }
  cmd_vel.twist.linear.x = cmd_vx;
  cmd_vel.twist.linear.y = cmd_vy;
  cmd_vel.twist.angular.z = angular_vel;

  prev_cmd_vel_ = cmd_vel; 
  has_prev_cmd_vel_ = true; 
  return cmd_vel;
}

nav_msgs::msg::Path OmniPidPursuitController::cropGlobalPlanToLocal(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  nav_msgs::msg::Path local_plan;
  local_plan.header = global_plan_.header; 
  local_plan.poses.reserve(std::distance(transformation_begin, transformation_end));
  
  for (auto it = transformation_begin; it != transformation_end; ++it) {
    local_plan.poses.push_back(*it);
  }
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  local_path_pub_->publish(local_plan);

  if (local_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  // RCLCPP_INFO(logger_, "裁剪路径: 全局[%lu] -> 局部[%lu] (坐标系: %s)",
  //             global_plan_.poses.size() + local_plan.poses.size(), // 加回来是因为刚才 erase 了
  //             local_plan.poses.size(),
  //             local_plan.header.frame_id.c_str());

  return local_plan;
}
// geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
//   const geometry_msgs::msg::PoseStamped & pose,
//   const geometry_msgs::msg::Twist & velocity,
//   nav2_core::GoalChecker * /*goal_checker*/)
// {
//   std::lock_guard<std::mutex> lock_reinit(mutex_);
//   geometry_msgs::msg::TwistStamped cmd_vel;
//   cmd_vel.header = pose.header;

//   // 1. 获取 Nav2 标准局部路径
//   auto local_plan = cropGlobalPlanToLocal(pose);
//   if (global_plan_.poses.empty()) {
//     return cmd_vel;
//   }

//   static bool first_plan = true;
//   static rclcpp::Time last_path_stamp;
  
//   rclcpp::Time current_path_stamp(global_plan_.header.stamp);
//   // bool path_is_new = (first_plan) || 
//   //                    (fabs((current_path_stamp - last_path_stamp).seconds()) > 0.5);
//   bool path_is_new = first_plan || minco_tracker_->isFinished();
//   if (path_is_new) {
//     RCLCPP_INFO(logger_, "Generating new MINCO trajectory (map frame)...");
    
//     bool success = minco_tracker_->setPath(local_plan, velocity);
//     if (!success) {
//       RCLCPP_WARN(logger_, "MINCO setPath failed!");
//       return cmd_vel;
//     }
//     last_path_stamp = global_plan_.header.stamp;
//     first_plan = false;
//     minco_tracker_->resetTime();
//   }

//   double dt = control_duration_; 
//   Eigen::Vector2d des_vel_map, des_acc_map;
//   minco_tracker_->advanceAndGetCmd(dt, des_vel_map, des_acc_map);
//   double t = minco_tracker_->getCurrentTime();
//   double total_t = minco_tracker_->getTotalTime(); // 你需要在 MincoTracker 里加这个函数
//   std::cout << "[DEBUG] 时间: t=" << t << " / total_t=" << total_t << std::endl;

//   t = std::min(t, total_t);

//   Eigen::Vector2d des_pos_map = minco_tracker_->getDesiredPos(t);
//   std::cout << "pos" << des_pos_map << std::endl;

//   double robot_x = pose.pose.position.x;
//   double robot_y = pose.pose.position.y;

//   tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
//   tf2::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);

//   // 1. 计算 map 下的误差
//   double dx_map = des_pos_map.x() - robot_x;
//   double dy_map = des_pos_map.y() - robot_y;

//   // 2. 旋转到 base_link
//   double c = cos(yaw);
//   double s = sin(yaw);
//   double des_x_robot =  c * dx_map + s * dy_map;
//   double des_y_robot = -s * dx_map + c * dy_map;

//   // 3. 速度也旋转
//   double des_vx_robot =  c * des_vel_map.x() + s * des_vel_map.y();
//   double des_vy_robot = -s * des_vel_map.x() + c * des_vel_map.y();

//   // ==============================
//   // 现在 des_x_robot / des_y_robot 是正常的小数值！
//   // ==============================
//   std::cout << "[正常] Robot frame: x=" << des_x_robot << ", y=" << des_y_robot << std::endl;

//   // 反馈控制
//   double kp = 1.0;
//   double fb_vx = kp * des_x_robot;
//   double fb_vy = kp * des_y_robot;

//   double final_vx = des_vx_robot + fb_vx;
//   double final_vy = des_vy_robot + fb_vy;

//   // 6. 简单限速
//   double speed = std::hypot(final_vx, final_vy);
//   if (speed > v_linear_max_) {
//     double scale = v_linear_max_ / speed;
//     final_vx *= scale;
//     final_vy *= scale;
//   }

//   auto smooth_path = minco_tracker_->getSmoothedPath();
//   smooth_path.header.stamp = clock_->now();
//   smoothed_path_pub_->publish(smooth_path);

//   // 7. 赋值输出
//   // cmd_vel.twist.linear.x = final_vx;
//   // cmd_vel.twist.linear.y = final_vy;
//   cmd_vel.twist.angular.z = 0.0; // 全向车不需要旋转车头

//   return cmd_vel;
// }
std::vector<Eigen::Vector2d> OmniPidPursuitController::samplePathToRefSeq(
  const std::vector<geometry_msgs::msg::PoseStamped> &path_poses,
  int Np)
{
  std::vector<Eigen::Vector2d> ref_seq;
  // 1. 将ROS路径点转换为Eigen向量，同时记录每个点到机器人的距离（本体帧原点(0,0)）
  std::vector<std::pair<Eigen::Vector2d, double>> pose_with_dist;
  for (const auto &p : path_poses) {
    Eigen::Vector2d pt(p.pose.position.x, p.pose.position.y);
    double dist = pt.norm();  // 本体帧下到原点的欧式距离（hypot(x,y)等价，更简洁）
    pose_with_dist.emplace_back(pt, dist);
  }

  // 2. 按距离**升序排序**：近→远，匹配机器人前进路径顺序
  std::sort(pose_with_dist.begin(), pose_with_dist.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; });

  // 3. 截取前Np个点，填充到参考序列
  int take_num = std::min((int)pose_with_dist.size(), Np);
  for (int i = 0; i < take_num; ++i) {
    ref_seq.push_back(pose_with_dist[i].first);
  }

  // 4. 鲁棒处理：若路径点不足Np，补最后一个点（保证参考序列长度为Np）
  if (ref_seq.size() < Np && !ref_seq.empty()) {
    Eigen::Vector2d last_pt = ref_seq.back();
    while (ref_seq.size() < Np) {
      ref_seq.push_back(last_pt);
    }
  }

  return ref_seq;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path) { 
  global_plan_ = path; 
  // minco_tracker_->reset();
}

void OmniPidPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
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
  // std::cout << "Original transformed plan size: " << transformed_plan.poses.size() << std::endl;
  bool robot_in_obstacle = false;
  {
    auto costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    unsigned int mx, my;
    if (costmap->worldToMap(0.0, 0.0, mx, my)) {
      unsigned char robot_cost = costmap->getCost(mx, my);
      robot_in_obstacle = (robot_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
  }

  if (!robot_in_obstacle && transformed_plan.poses.size() > 2) {
    transformed_plan.poses = removeCornerPts(transformed_plan.poses);
    // std::cout << "Removed corner points, remaining poses: " << transformed_plan.poses.size() << std::endl;
  }
  // std::cout << "Transformed plan size: " << transformed_plan.poses.size() << std::endl;
  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    std::cout << "Transformed plan is empty after pruning. Returning zero velocity." << std::endl;
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}
std::vector<geometry_msgs::msg::PoseStamped> OmniPidPursuitController::removeCornerPts(
  const std::vector<geometry_msgs::msg::PoseStamped> &path) 
{
  if (path.size() < 4)
      return path;
  const double OBSTACLE_NEAR_DISTANCE = 0.3;
  // cut zigzag segment
  std::vector<geometry_msgs::msg::PoseStamped> optimized_path;
  geometry_msgs::msg::PoseStamped pose1 = path[0];
  geometry_msgs::msg::PoseStamped pose2 = path[1];
  geometry_msgs::msg::PoseStamped prev_pose = pose1;
  optimized_path.push_back(pose1);
  double cost1, cost2, cost3;

  if (!checkLineCollision(pose1, pose2))
      cost1 = euclideanDistance(pose1, pose2);
  else
      cost1 = std::numeric_limits<double>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
      pose1 = path[i];
      pose2 = path[i + 1];
      const double skip_distance = euclideanDistance(prev_pose, pose2);
      if (skip_distance > max_skip_distance) {
          optimized_path.push_back(path[i]);
          cost1 = euclideanDistance(pose1, pose2);
          prev_pose = pose1;
          continue;
      }
      double dist_to_robot = hypot(pose1.pose.position.x, pose1.pose.position.y);

      if (!checkLineCollision(pose1, pose2))
          cost2 = euclideanDistance(pose1, pose2);
      else
          // cost2 = euclideanDistance(pose1, pose2);
          cost2 = std::numeric_limits<double>::infinity();

      if (!checkLineCollision(prev_pose, pose2))
          cost3 = euclideanDistance(prev_pose, pose2);
      else
          // cost3 = euclideanDistance(pose1, pose2);
          cost3 = std::numeric_limits<double>::infinity();
      // std::cout << "cost1: " << cost1 << " cost2: " << cost2 << " cost3: " << cost3 << std::endl;
      if (cost3 < cost1 + cost2) {
          cost1 = cost3;  
      } else {
          optimized_path.push_back(path[i]);
          cost1 = euclideanDistance(pose1, pose2);
          prev_pose = pose1;
      }
  }
  // for (size_t i = 0; i < optimized_path.size(); ++i) {
  //   const auto &p = optimized_path[i].pose.position;
  //   std::cout << "  [" << i << "]: (" 
  //             << std::fixed << std::setprecision(3) 
  //             << p.x << ", " << p.y << ")" << std::endl;
  // }
  optimized_path.push_back(path.back());
  // auto final_path = smoothPathCorners(
  //   optimized_path,
  //   0.3,    // 平滑半径（米）
  //   10,       // 插5个点足够顺滑
  //   150.0,    // 150度以内才认为是拐角
  //   3
  // );
  // return final_path;
  return optimized_path;
}
std::vector<geometry_msgs::msg::PoseStamped> OmniPidPursuitController::smoothPathCorners(
  const std::vector<geometry_msgs::msg::PoseStamped>& path,
  double smooth_radius = 0.25,
  int num_interpolation = 5,
  double angle_tol_deg = 150.0,
  int skip_points = 5)
{
  if (path.size() < 3)
      return path;

  std::vector<geometry_msgs::msg::PoseStamped> smoothed;
  const double ANGLE_TOL_RAD = angle_tol_deg * M_PI / 180.0;

  std::vector<bool> in_corner_region(path.size(), false);
  std::vector<bool> is_corner_center(path.size(), false);

  for (size_t i = skip_points; i < path.size() - skip_points; ++i)
  {
    const auto& p0 = path[i - skip_points].pose.position;
    const auto& p1 = path[i].pose.position;
    const auto& p2 = path[i + skip_points].pose.position;

    double dx1 = p0.x - p1.x;
    double dy1 = p0.y - p1.y;
    double dx2 = p2.x - p1.x;
    double dy2 = p2.y - p1.y;

    double len1 = hypot(dx1, dy1);
    double len2 = hypot(dx2, dy2);
    if (len1 < 0.01 || len2 < 0.01) continue;

    double ux1 = dx1 / len1;
    double uy1 = dy1 / len1;
    double ux2 = dx2 / len2;
    double uy2 = dy2 / len2;

    double dot = ux1 * ux2 + uy1 * uy2;
    dot = std::clamp(dot, -1.0, 1.0);
    double angle = acos(dot);

    if (angle < ANGLE_TOL_RAD)
    {
      is_corner_center[i] = true;
      for (int j = -skip_points; j <= skip_points; ++j)
      {
        size_t idx = i + j;
        if (idx >= 0 && idx < path.size())
          in_corner_region[idx] = true;
      }
    }
  }

  size_t i = 0;
  while (i < path.size())
  {
    // ==============================================
    // ✅ 非拐点区域：直接保留所有原始点
    // ==============================================
    if (!in_corner_region[i])
    {
      smoothed.push_back(path[i]);
      i++;
      continue;
    }

    size_t corner_center = i;
    while (corner_center < path.size() && !is_corner_center[corner_center])
      corner_center++;

    if (corner_center >= path.size() - skip_points)
    {

      while (i < path.size())
      {
        smoothed.push_back(path[i]);
        i++;
      }
      break;
    }

    size_t idx0 = corner_center - skip_points;
    size_t idx2 = corner_center + skip_points;

    const auto& p0 = path[idx0].pose.position;
    const auto& p1 = path[corner_center].pose.position;
    const auto& p2 = path[idx2].pose.position;

    double dx1 = p0.x - p1.x;
    double dy1 = p0.y - p1.y;
    double dx2 = p2.x - p1.x;
    double dy2 = p2.y - p1.y;

    double len1 = hypot(dx1, dy1);
    double len2 = hypot(dx2, dy2);
    if (len1 < 0.01 || len2 < 0.01)
    {

      while (i <= idx2 && i < path.size())
      {
        smoothed.push_back(path[i]);
        i++;
      }
      continue;
    }

    double ux1 = dx1 / len1;
    double uy1 = dy1 / len1;
    double ux2 = dx2 / len2;
    double uy2 = dy2 / len2;

    double d = smooth_radius;

    auto start = path[corner_center];
    start.pose.position.x = p1.x - ux1 * d;
    start.pose.position.y = p1.y - uy1 * d;

    auto end = path[corner_center];
    end.pose.position.x = p1.x + ux2 * d;
    end.pose.position.y = p1.y + uy2 * d;

    smoothed.push_back(path[idx0]);

    for (int j = 1; j <= num_interpolation; ++j)
    {
      double t = (double)j / (num_interpolation + 1);
      geometry_msgs::msg::PoseStamped pt;
      pt.header = path[corner_center].header;
      pt.pose.position.x = start.pose.position.x * (1 - t) + end.pose.position.x * t;
      pt.pose.position.y = start.pose.position.y * (1 - t) + end.pose.position.y * t;
      pt.pose.orientation = path[corner_center].pose.orientation;
      smoothed.push_back(pt);
    }

    i = idx2 + 1;
  }

  return smoothed;
}
double OmniPidPursuitController::euclideanDistance(
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2)
{
  double dx = p1.pose.position.x - p2.pose.position.x;
  double dy = p1.pose.position.y - p2.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool OmniPidPursuitController::checkLineCollision(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end)
{
  auto costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  const double resolution = costmap->getResolution();
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  double x0 = start.pose.position.x;
  double y0 = start.pose.position.y;
  double x1 = end.pose.position.x;
  double y1 = end.pose.position.y;

  auto world_to_grid_base_link = [&](double x, double y) -> std::pair<int, int> {
      int mx = static_cast<int>(x / resolution + size_x / 2.0);
      int my = static_cast<int>(y / resolution + size_y / 2.0);
      return {mx, my};
  };

  auto [grid_x0, grid_y0] = world_to_grid_base_link(x0, y0);
  auto [grid_x1, grid_y1] = world_to_grid_base_link(x1, y1);

  // 范围检查
  auto is_in_range = [&](int gx, int gy) {
      return (gx >= 0 && gx < size_x && gy >= 0 && gy < size_y);
  };

  if (!is_in_range(grid_x0, grid_y0) || !is_in_range(grid_x1, grid_y1)) {
      RCLCPP_DEBUG(logger_, "Point out of range -> skip check");
      return true; // 越界认为无碰撞
  }

  // Bresenham 算法
  int dx = std::abs(grid_x1 - grid_x0);
  int dy = std::abs(grid_y1 - grid_y0);
  int sx = (grid_x0 < grid_x1) ? 1 : -1;
  int sy = (grid_y0 < grid_y1) ? 1 : -1;
  int err = dx - dy;

  int x = grid_x0;
  int y = grid_y0;

  const int max_iter = 200;
  int iter = 0;

  while (iter < max_iter) {
      iter++;

      if (is_in_range(x, y)) {
          size_t index = static_cast<size_t>(y) * static_cast<size_t>(size_x) + static_cast<size_t>(x);
          
          if (index < static_cast<size_t>(size_x * size_y)) {
              unsigned char cost = costmap->getCharMap()[index]; 
              
              // RCLCPP_DEBUG(logger_, "Grid(%d, %d) -> cost=%d", x, y, (int)cost);

              if (cost >= 120) {
                  // RCLCPP_WARN(logger_, "COLLISION DETECTED at Grid(%d, %d), cost = %d", x, y, (int)cost);
                  return true;
              }
          }
      }

      if (x == grid_x1 && y == grid_y1) {
          break;
      }

      int e2 = 2 * err;
      if (e2 > -dy) {
          err -= dy;
          x += sx;
      }
      if (e2 < dx) {
          err += dx;
          y += sy;
      }
  }

  return false;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
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

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
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

double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

bool OmniPidPursuitController::transformPose(
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

bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path & path)
{
  auto costmap = costmap_ros_->getCostmap();
  for (const auto & pose_stamped : path.poses) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
      }
    } else {
      return false;
    }
  }
  return false;
}
bool OmniPidPursuitController::isDirectPathSafeToTarget(
  const geometry_msgs::msg::PoseStamped & target_pose) const
{
  auto costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  const double resolution = costmap->getResolution();
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  const double x0 = 0.0;
  const double y0 = 0.0;
  const double x1 = target_pose.pose.position.x;
  const double y1 = target_pose.pose.position.y;

  const double distance = std::hypot(x1 - x0, y1 - y0);
  if (distance < 1e-6) {
    return true;
  }

  const double sample_step = resolution * 0.5;
  const int sample_count = std::max(1, static_cast<int>(distance / sample_step));

  auto base_to_grid = [&](double x, double y, int & gx, int & gy) -> bool {
    gx = static_cast<int>(x / resolution + size_x / 2.0);
    gy = static_cast<int>(y / resolution + size_y / 2.0);

    return gx >= 0 && gx < size_x && gy >= 0 && gy < size_y;
  };

  for (int i = 0; i <= sample_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
    const double x = x0 + ratio * (x1 - x0);
    const double y = y0 + ratio * (y1 - y0);

    int gx = 0;
    int gy = 0;

    if (!base_to_grid(x, y, gx, gy)) {
      return false;
    }

    const size_t index =
      static_cast<size_t>(gy) * static_cast<size_t>(size_x) + static_cast<size_t>(gx);

    if (index >= static_cast<size_t>(size_x * size_y)) {
      return false;
    }

    const unsigned char cost = costmap->getCharMap()[index];

    if (static_cast<int>(cost) > 200) {
      return false;
    }
  }

  return true;
}
double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance

  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    double vel_to_use = 0.0;
    if (has_prev_cmd_vel_) {
      // 计算历史速度的合成线速度（vx, vy的模长）
      vel_to_use = hypot(
        prev_cmd_vel_.twist.linear.x,
        prev_cmd_vel_.twist.linear.y
      );
    } else {
      // 第一次运行，用当前反馈速度
      vel_to_use = hypot(speed.linear.x, speed.linear.y);
    }
    lookahead_dist = vel_to_use * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel = unbounded_vel;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void OmniPidPursuitController::applyApproachVelocityScaling_mpc(
  const nav_msgs::msg::Path & path, double &vx, double &vy)
{
  double linear_vel = hypot(vx, vy);
  double original_linear_vel = linear_vel;
  if (linear_vel < 1e-6) {
    return;
  }

  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel = unbounded_vel;
  }
  linear_vel = std::min(linear_vel, approach_vel);

  double scale_ratio = linear_vel / original_linear_vel;
  vx *= scale_ratio;
  vy *= scale_ratio;
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  double curvature =
  calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);
  RCLCPP_DEBUG(logger_, "Curvature: %.3f", curvature);
  if(slow && curvature <= large_slow_-0.3) slow = false;
  if(!slow && curvature >= large_slow_) slow = true;
  double scaled_linear_vel = linear_vel;


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
  linear_vel = std::clamp(linear_vel,last_velocity_scaling_factor_-lower_speed_,last_velocity_scaling_factor_+lower_speed_);
  // std::lock_guard<std::mutex> lock(sm_mutex);
  last_velocity_scaling_factor_ = linear_vel;
}

void OmniPidPursuitController::applyCurvatureLimitation_mpc(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & vx, double & vy)
{
  double linear_vel = hypot(vx, vy);
  double original_linear_vel = linear_vel;
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);
  RCLCPP_DEBUG(logger_, "Curvature: %.3f", curvature);
  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;

    if(curvature > 1.0 && last_velocity_scaling_factor_ > last_vel_){
      scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -lower_speed_ ,
                                        lower_speed_ );
    }  
    else if(last_velocity_scaling_factor_ > target_scaled_vel){
      scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
    }                                    
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);
  RCLCPP_DEBUG(logger_, "Scaled linear vel: %.3f", scaled_linear_vel);
  linear_vel = std::min(linear_vel, scaled_linear_vel);
  double scale_ratio = linear_vel / original_linear_vel;
  vx *= scale_ratio;  
  vy *= scale_ratio;
  // std::lock_guard<std::mutex> lock(sm_mutex);
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
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
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
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

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
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

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
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

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  bool need_reinit_controller = false;  // 标记是否需要重新初始化控制器
  bool new_use_mpc = use_mpc_control_;  // 暂存新的MPC启用状态


  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      // 原有PID参数处理
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
        if (!use_mpc_control_ && move_pid_) {
          // move_pid_->setKp(translation_kp_); // 实时更新PID参数
        }
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
        if (!use_mpc_control_ && move_pid_) {
          // move_pid_->setKi(translation_ki_);
        }
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
        if (!use_mpc_control_ && move_pid_) {
          // move_pid_->setKd(translation_kd_);
        }
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
        if (heading_pid_) {
          // heading_pid_->setKp(rotation_kp_);
        }
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
        if (heading_pid_) {
          // heading_pid_->setKi(rotation_ki_);
        }
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
        if (heading_pid_) {
          // heading_pid_->setKd(rotation_kd_);
        }
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
        if (!use_mpc_control_ && move_pid_) {
          // move_pid_->setMaxOutput(v_linear_max_);
        }
        if (use_mpc_control_ && mpc_controller_) {
          mpc_controller_->setVelocityLimits(v_linear_min_, v_linear_max_);
        }
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
        if (!use_mpc_control_ && move_pid_) {
          // move_pid_->setMinOutput(v_linear_min_);
        }
        if (use_mpc_control_ && mpc_controller_) {
          mpc_controller_->setVelocityLimits(v_linear_min_, v_linear_max_);
        }
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
        if (heading_pid_) {
          // heading_pid_->setMaxOutput(v_angular_max_);
        }
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
        if (heading_pid_) {
          // heading_pid_->setMinOutput(v_angular_min_);
        }
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      } else if (name == plugin_name_ + ".last_vel") {
        last_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lower_speed") {
        lower_speed_ = parameter.as_double();
      }
      // MPC权重参数动态更新
      else if (name == plugin_name_ + ".mpc_Q_x") {
        mpc_Q_x = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          Eigen::Matrix2d Q, R, R_delta;
          Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
          R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
          R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
          mpc_controller_->initWeights(Q, R, R_delta);
        }
      } else if (name == plugin_name_ + ".mpc_Q_y") {
        mpc_Q_y = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          Eigen::Matrix2d Q, R, R_delta;
          Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
          R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
          R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
          mpc_controller_->initWeights(Q, R, R_delta);
        }
      } else if (name == plugin_name_ + ".mpc_R_vx") {
        mpc_R_vx = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          Eigen::Matrix2d Q, R, R_delta;
          Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
          R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
          R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
          mpc_controller_->initWeights(Q, R, R_delta);
        }
      } else if (name == plugin_name_ + ".mpc_R_vy") {
        mpc_R_vy = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          Eigen::Matrix2d Q, R, R_delta;
          Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
          R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
          R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
          mpc_controller_->initWeights(Q, R, R_delta);
        }
      } else if (name == plugin_name_ + ".mpc_Rdelta_vx") {
        mpc_Rdelta_vx = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          Eigen::Matrix2d Q, R, R_delta;
          Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
          R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
          R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
          mpc_controller_->initWeights(Q, R, R_delta);
        }
      } else if (name == plugin_name_ + ".mpc_Rdelta_vy") {
        mpc_Rdelta_vy = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          Eigen::Matrix2d Q, R, R_delta;
          Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
          R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
          R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
          mpc_controller_->initWeights(Q, R, R_delta);
        }
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      }
      else if (name == plugin_name_ + ".enable_sg_filter") {
        impl_->enable_sg_filter_ = parameter.as_bool();
        if (impl_->enable_sg_filter_) {
          impl_->linear_vel_filter_->reset();
          impl_->angular_vel_filter_->reset();
        }
      }
      // 核心：算法切换参数
      else if (name == plugin_name_ + ".use_mpc_control") {
        new_use_mpc = parameter.as_bool();
        if (new_use_mpc != use_mpc_control_) {
          need_reinit_controller = true; // 标记需要重新初始化控制器
        }
      }
    } 
    // SG滤波整型参数（窗口大小/多项式阶数）
    else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".sg_window_size") {
        impl_->sg_window_size_ = parameter.as_int();
        impl_->linear_vel_filter_->setWindowSize(impl_->sg_window_size_);
        impl_->angular_vel_filter_->setWindowSize(impl_->sg_window_size_);
      } else if (name == plugin_name_ + ".sg_poly_order") {
        impl_->sg_poly_order_ = parameter.as_int();
        impl_->linear_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
        impl_->angular_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
      }
      // MPC预测/控制时域
      else if (name == plugin_name_ + ".mpc_Np") {
        mpc_Np_ = parameter.as_int();
        if (use_mpc_control_ && mpc_controller_) {
          mpc_controller_->setNp(mpc_Np_);
        }
      } else if (name == plugin_name_ + ".mpc_Nc") {
        mpc_Nc_ = parameter.as_int();
        if (use_mpc_control_ && mpc_controller_) {
          mpc_controller_->setNc(mpc_Nc_);
        }
      }
    }
  }

  // 2. 算法切换：重新初始化控制器
  if (need_reinit_controller) {
    auto node = node_.lock();
    if (!node) {
      RCLCPP_ERROR(logger_, "Failed to lock node when switching control mode!");
      result.successful = false;
      result.reason = "Node lock failed";
      return result;
    }

    // 更新全局MPC启用状态
    use_mpc_control_ = new_use_mpc;

    if (use_mpc_control_) {

      move_pid_.reset();

      mpc_controller_ = std::make_unique<OmniMpcController>(
        control_duration_,  
        mpc_Np_,            
        mpc_Nc_,          
        v_linear_min_,      
        v_linear_max_       
      );
      // 重新设置MPC权重
      Eigen::Matrix2d Q, R, R_delta;
      Q << mpc_Q_x, 0.0, 0.0, mpc_Q_y;
      R << mpc_R_vx, 0.0, 0.0, mpc_R_vy;
      R_delta << mpc_Rdelta_vx, 0.0, 0.0, mpc_Rdelta_vy;
      mpc_controller_->initWeights(Q, R, R_delta);
      RCLCPP_INFO(logger_, "Successfully switched to MPC control mode");
    } else {
      // 切换到PID模式：释放MPC资源，初始化PID
      mpc_controller_.reset();
      // 重新初始化PID控制器
      move_pid_ = std::make_shared<PID>(
        control_duration_, v_linear_max_, v_linear_min_, 
        translation_kp_, translation_kd_, translation_ki_);
      heading_pid_ = std::make_shared<PID>(
        control_duration_, v_angular_max_, v_angular_min_, 
        rotation_kp_, rotation_kd_, rotation_ki_);
      // 更新滤波器参数
      impl_->linear_vel_filter_->setWindowSize(impl_->sg_window_size_);
      impl_->linear_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
      impl_->angular_vel_filter_->setWindowSize(impl_->sg_window_size_);
      impl_->angular_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
      RCLCPP_INFO(logger_, "Successfully switched to PID control mode");
    }
  }

  // 3. 返回参数设置结果
  result.successful = true;
  result.reason = "Parameters updated successfully";
  return result;
}
};  // namespace pb_omni_pid_pursuit_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)