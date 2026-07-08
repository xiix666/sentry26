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
    node, plugin_name_ + ".mpc_S_x", rclcpp::ParameterValue(15.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_S_y", rclcpp::ParameterValue(15.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_S_vx", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_S_vy", rclcpp::ParameterValue(0.0));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Q_x", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Q_y", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Q_vx", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_Q_vy", rclcpp::ParameterValue(0.5));

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
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_ref_speed", rclcpp::ParameterValue(2.5));
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
  node->get_parameter(plugin_name_ + ".mpc_S_x", mpc_S_x);
  node->get_parameter(plugin_name_ + ".mpc_S_y", mpc_S_y);
  node->get_parameter(plugin_name_ + ".mpc_S_vx", mpc_S_vx);
  node->get_parameter(plugin_name_ + ".mpc_S_vy", mpc_S_vy);

  node->get_parameter(plugin_name_ + ".mpc_Q_x", mpc_Q_x);
  node->get_parameter(plugin_name_ + ".mpc_Q_y", mpc_Q_y);
  node->get_parameter(plugin_name_ + ".mpc_Q_vx", mpc_Q_vx);
  node->get_parameter(plugin_name_ + ".mpc_Q_vy", mpc_Q_vy);

  node->get_parameter(plugin_name_ + ".mpc_R_vx", mpc_R_vx);
  node->get_parameter(plugin_name_ + ".mpc_R_vy", mpc_R_vy);
  node->get_parameter(plugin_name_ + ".mpc_Rdelta_vx", mpc_Rdelta_vx);
  node->get_parameter(plugin_name_ + ".mpc_Rdelta_vy", mpc_Rdelta_vy);

  node->get_parameter(plugin_name_ + ".mpc_Np", mpc_Np_);
  node->get_parameter(plugin_name_ + ".mpc_Nc", mpc_Nc_);
  node->get_parameter(plugin_name_ + ".min_dist", min_dist_);
  node->get_parameter(plugin_name_ + ".acc_max", acc_max_);
  node->get_parameter(plugin_name_ + ".mpc_ref_speed", mpc_ref_speed_);

  declare_parameter_if_not_declared(
  node, plugin_name_ + ".mpc_curve_a_lat_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_curve_min_speed", rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_curve_lookahead_dist", rclcpp::ParameterValue(1.2));
    control_duration_ = 1.0 / control_frequency;
  node->get_parameter(plugin_name_ + ".mpc_curve_a_lat_max", mpc_curve_a_lat_max_);
  node->get_parameter(plugin_name_ + ".mpc_curve_min_speed", mpc_curve_min_speed_);
  node->get_parameter(plugin_name_ + ".mpc_curve_lookahead_dist", mpc_curve_lookahead_dist_);
  // minco_tracker_ = std::make_unique<minco_nav2::MincoTracker>();
  // minco_tracker_->setParams(v_linear_max_,acc_max_,control_duration_);
  if (use_mpc_control_) {
    // MPC模式：初始化MPC控制器
    mpc_controller_ = std::make_unique<OmniMpcController>(
    control_duration_,   // Ts
    mpc_Np_,             // Np
    mpc_Nc_,             // Nc
    -acc_max_,           // a_min
    acc_max_,            // a_max
    v_linear_min_,       // v_min
    v_linear_max_        // v_max
    );

    updateMpcWeights();
  } else {
    impl_->linear_vel_filter_->setWindowSize(impl_->sg_window_size_);
    impl_->linear_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
    impl_->angular_vel_filter_->setWindowSize(impl_->sg_window_size_);
    impl_->angular_vel_filter_->setPolyOrder(impl_->sg_poly_order_);
  }
  move_pid_ = std::make_shared<PID>(
    control_duration_, 
    v_linear_max_, 
    v_linear_min_, 
    translation_kp_, 
    translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_,
    v_angular_max_,
    v_angular_min_,
    rotation_kp_,
    rotation_kd_,
    rotation_ki_);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  
  smooth_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(  //如果不用smooth_server的话，这个订阅可以去掉
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

}
void OmniPidPursuitController::updateMpcWeights()
{
  if (!mpc_controller_) {
    return;
  }

  Eigen::Matrix4d S = Eigen::Matrix4d::Zero();
  S(0, 0) = mpc_S_x;
  S(1, 1) = mpc_S_y;
  S(2, 2) = mpc_S_vx;
  S(3, 3) = mpc_S_vy;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = mpc_Q_x;
  Q(1, 1) = mpc_Q_y;
  Q(2, 2) = mpc_Q_vx;
  Q(3, 3) = mpc_Q_vy;

  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = mpc_R_vx;
  R(1, 1) = mpc_R_vy;

  Eigen::Matrix2d R_delta = Eigen::Matrix2d::Zero();
  R_delta(0, 0) = mpc_Rdelta_vx;
  R_delta(1, 1) = mpc_Rdelta_vy;

  mpc_controller_->initWeights(S, Q, R, R_delta);
}
void OmniPidPursuitController::smoothedVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(sm_mutex);
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
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  double lin_dist = 0.0;
  double theta_dist = 0.0;
  double angle_to_goal = 0.0;

  double cmd_vx = 0.0;
  double cmd_vy = 0.0;
  double lin_vel = 0.0;
  double angular_vel = 0.0;

  std::vector<geometry_msgs::msg::PoseStamped> valid_path_poses;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;

  auto transformed_plan = transformGlobalPlan(pose);

  const double lookahead_dist = getLookAheadDistance(velocity);
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

  const bool carrot_valid =
    std::isfinite(carrot_pose.pose.position.x) &&
    std::isfinite(carrot_pose.pose.position.y);

  if (!carrot_valid) {
    RCLCPP_WARN(logger_, "Carrot pose invalid (NaN/Inf), return zero vel");
    return cmd_vel;
  }

  const bool direct_drive_request = (rm_task_value_.load() == 1);
  bool direct_drive_mode = false;

  if (direct_drive_request) {
    const auto & target_pose = transformed_plan.poses.back();

    const double target_x = target_pose.pose.position.x;
    const double target_y = target_pose.pose.position.y;

    const bool target_valid =
      std::isfinite(target_x) && std::isfinite(target_y);

    if (target_valid && isDirectPathSafeToTarget(target_pose)) {
      direct_drive_mode = true;
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Direct drive path unsafe or target invalid, fallback to normal controller");
    }
  }

  const auto * carrot_visual_pose =
    direct_drive_mode ? &transformed_plan.poses.back() : &carrot_pose;

  const bool should_publish_carrot =
    !use_mpc_control_ || direct_drive_mode;

  if (should_publish_carrot) {
    carrot_pub_->publish(createCarrotMsg(*carrot_visual_pose));
  }

  if (use_mpc_control_ && !direct_drive_mode) {
    for (const auto & p : transformed_plan.poses) {
      const double dist = std::hypot(
        p.pose.position.x,
        p.pose.position.y);

      if (dist >= min_dist_ && dist <= 8.0) {
        valid_path_poses.push_back(p);
      }
    }

    if (valid_path_poses.empty()) {
      for (const auto & p : transformed_plan.poses) {
        const double dist = std::hypot(
          p.pose.position.x,
          p.pose.position.y);

        if (dist > 0.0 && dist <= 8.0) {
          valid_path_poses.push_back(p);
        }
      }
    }

    if (!valid_path_poses.empty()) {
      lin_dist = std::hypot(
        valid_path_poses[0].pose.position.x,
        valid_path_poses[0].pose.position.y);
    }
  }
  auto runProportionalPursuit = [&]() {
    lin_dist = std::hypot(
      carrot_pose.pose.position.x,
      carrot_pose.pose.position.y);

    theta_dist = std::atan2(
      carrot_pose.pose.position.y,
      carrot_pose.pose.position.x);

    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);

    if (use_rotate_to_heading_) {
      if (std::fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
        lin_dist = 0.0;
      }
    }

    if (lin_dist > 0.0) {

      lin_vel = move_pid_->calculate(lin_dist, 0);
      if (!std::isfinite(lin_vel)) {
        RCLCPP_WARN(logger_, "PID returned NaN/Inf, using zero vel");
        lin_vel = 0.0;
      }

      applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
      applyApproachVelocityScaling(transformed_plan, lin_vel);

      cmd_vx = lin_vel * std::cos(theta_dist);
      cmd_vy = lin_vel * std::sin(theta_dist);

      RCLCPP_DEBUG(
        logger_,
        "Proportional/PID pursuit output: lin_vel=%.3f, vx=%.3f, vy=%.3f",
        lin_vel, cmd_vx, cmd_vy);
    }

    if (impl_->enable_sg_filter_ && lin_dist > 0.0) {
      const double filtered_lin_vel = impl_->linear_vel_filter_->filter(lin_vel);

      cmd_vx = filtered_lin_vel * std::cos(theta_dist);
      cmd_vy = filtered_lin_vel * std::sin(theta_dist);

      RCLCPP_DEBUG(logger_, "SG filtered vel: %.3f", filtered_lin_vel);
    }

    angular_vel =
      enable_rotation_ && heading_pid_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;
  };
  if (direct_drive_mode) {
    const auto & target_pose = transformed_plan.poses.back();

    const double target_x = target_pose.pose.position.x;
    const double target_y = target_pose.pose.position.y;

    lin_dist = std::hypot(target_x, target_y);
    theta_dist = std::atan2(target_y, target_x);

    if (lin_dist > 1e-6) {

      lin_vel = translation_kp_ * lin_dist;

      lin_vel = std::clamp(lin_vel, 0.0, v_linear_max_);

      cmd_vx = lin_vel * std::cos(theta_dist);
      cmd_vy = lin_vel * std::sin(theta_dist);
    }

    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    angular_vel =
      enable_rotation_ && heading_pid_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;
  }
  else if (use_mpc_control_) {
    const auto & target_pose = transformed_plan.poses.back();

    const double target_x = target_pose.pose.position.x;
    const double target_y = target_pose.pose.position.y;

    const bool target_valid =
      std::isfinite(target_x) &&
      std::isfinite(target_y);

    const double target_dist =
      target_valid ? std::hypot(target_x, target_y) : std::numeric_limits<double>::infinity();

    const bool mpc_use_proportional_near_goal =
      target_valid &&
      target_dist <= approach_velocity_scaling_dist_;
    const double enter_dist = approach_velocity_scaling_dist_;
    const double exit_dist = approach_velocity_scaling_dist_ + 0.20;

    if (!mpc_near_goal_pursuit_mode_ && target_dist <= enter_dist) {
      mpc_near_goal_pursuit_mode_ = true;

      if (mpc_controller_) {
        mpc_controller_->reset();
      }
    }
    if (mpc_near_goal_pursuit_mode_ && target_dist >= exit_dist) {
      mpc_near_goal_pursuit_mode_ = false;

      if (mpc_controller_) {
        mpc_controller_->reset();
      }
    }
    if (mpc_near_goal_pursuit_mode_) {
      runProportionalPursuit();
    } 
    else {
      if (!transformed_plan.poses.empty() && mpc_controller_) {
        OmniMpcController::State x0;
        x0.setZero();

        x0(0) = 0.0;
        x0(1) = 0.0;
        x0(2) = velocity.linear.x;
        x0(3) = velocity.linear.y;

        if (!std::isfinite(x0(2))) {
          x0(2) = 0.0;
        }
        if (!std::isfinite(x0(3))) {
          x0(3) = 0.0;
        }

        double v_des = std::clamp(
          mpc_ref_speed_,
          min_approach_linear_velocity_,
          v_linear_max_);

        // auto mpc_ref_seq =
        //   samplePathToTimedMpcRefSeq(
        //     transformed_plan.poses,
        //     mpc_Np_,
        //     v_des);

        // mpc_controller_->setAccelerationLimits(-acc_max_, acc_max_);
        // mpc_controller_->setVelocityLimits(v_linear_min_, v_linear_max_);
        double v_ref_eff_for_constraint = v_des;

        auto mpc_ref_seq =
          samplePathToTimedMpcRefSeq(
            transformed_plan.poses,
            mpc_Np_,
            v_des,
            &v_ref_eff_for_constraint);

        mpc_controller_->setAccelerationLimits(-acc_max_, acc_max_);

        const double v_constraint_limit =
          std::clamp(
            v_ref_eff_for_constraint,
            std::max(0.0, mpc_curve_min_speed_),
            v_linear_max_);

        mpc_controller_->setVelocityLimits(-v_constraint_limit, v_constraint_limit);
        Eigen::Vector2d v_cmd =
          mpc_controller_->solveVelocityCommand(x0, mpc_ref_seq);

        cmd_vx = v_cmd.x();
        cmd_vy = v_cmd.y();
      }

      if (std::hypot(cmd_vx, cmd_vy) > 1e-6) {
        applyCurvatureLimitation_mpc(transformed_plan, carrot_pose, cmd_vx, cmd_vy);
        // 不要再调用 applyApproachVelocityScaling_mpc()
      }

      angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
      angular_vel =
        enable_rotation_ && heading_pid_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;
    }
  }
  else {
    runProportionalPursuit();
  }

  if (!std::isfinite(cmd_vx)) {
    cmd_vx = 0.0;
  }
  if (!std::isfinite(cmd_vy)) {
    cmd_vy = 0.0;
  }
  if (!std::isfinite(angular_vel)) {
    angular_vel = 0.0;
  }

  if (!use_mpc_control_ && has_prev_cmd_vel_) {
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

  const double max_linear_vel = 4.0;
  const double speed_mag = std::hypot(cmd_vx, cmd_vy);

  if (speed_mag > max_linear_vel && speed_mag > 1e-6) {
    const double scale = max_linear_vel / speed_mag;

    cmd_vx *= scale;
    cmd_vy *= scale;
  }

  cmd_vel.twist.linear.x = cmd_vx;
  cmd_vel.twist.linear.y = cmd_vy;
  // cmd_vel.twist.linear.x = 0.0;
  // cmd_vel.twist.linear.y = 0.0;
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
  
  nav_msgs::msg::Path global_segment;
  global_segment.header = global_plan_.header;
  global_segment.header.stamp = robot_pose.header.stamp;

  global_segment.poses.reserve(std::distance(transformation_begin, transformation_end));

  // Lambda to transform a PoseStamped from global frame to local
  // auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
  //   geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
  //   stamped_pose.header.frame_id = global_plan_.header.frame_id;
  //   stamped_pose.header.stamp = robot_pose.header.stamp;
  //   stamped_pose.pose = global_plan_pose.pose;
  //   transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
  //   transformed_pose.pose.position.z = 0.0;
  //   return transformed_pose;
  // };
  for (auto it = transformation_begin; it != transformation_end; ++it) {
    geometry_msgs::msg::PoseStamped p = *it;
    p.header.frame_id = global_plan_.header.frame_id;
    p.header.stamp = robot_pose.header.stamp;
    p.pose.position.z = 0.0;
    global_segment.poses.push_back(p);
  }

  if (global_segment.poses.empty()) {
    throw nav2_core::PlannerException("Resulting global segment has 0 poses in it.");
  }
  // Transform the near part of the global plan into the robot's frame of reference.
  // nav_msgs::msg::Path transformed_plan;
  // std::transform(
  //   transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
  //   transform_global_pose_to_local);
  // transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  // transformed_plan.header.stamp = robot_pose.header.stamp;
  // std::cout << "Original transformed plan size: " << transformed_plan.poses.size() << std::endl;
  bool robot_in_obstacle = false;
  {
    auto costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    geometry_msgs::msg::PoseStamped robot_costmap_pose;
    if (transformPose(costmap_ros_->getGlobalFrameID(), pose, robot_costmap_pose)) {
      unsigned int mx = 0;
      unsigned int my = 0;

      if (costmap->worldToMap(
            robot_costmap_pose.pose.position.x,
            robot_costmap_pose.pose.position.y,
            mx,
            my))
      {
        const unsigned char robot_cost = costmap->getCost(mx, my);
        robot_in_obstacle =
          robot_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      }
    }
  }

  if (!robot_in_obstacle && global_segment.poses.size() > 2) {
      global_segment.poses = removeCornerPts(global_segment.poses);
  }

  auto transform_global_pose_to_local =
    [&](const geometry_msgs::msg::PoseStamped & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose;
      geometry_msgs::msg::PoseStamped transformed_pose;

      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;

      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;

      return transformed_pose;
  };
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;
  transformed_plan.poses.reserve(global_segment.poses.size());

  std::transform(
    global_segment.poses.begin(),
    global_segment.poses.end(),
    std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);

  global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);

  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting transformed plan has 0 poses in it.");
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
  auto final_path = softSmoothPathCorners(
    optimized_path,
    0.20,   // 最大平滑半径
    8,      // 最大插值点数量
    15.0,    // 小于 15 度认为基本直线，不插
    60.0,   // 大于 60 度认为明显转弯，满权重
    0.25,   // 固定距离窗口，前后各看 0.35m
    0.05,   // 插值点最小间距
    2       // 少于 2 个插值点就不插
  );
  return final_path;
  // return optimized_path;
}

std::vector<geometry_msgs::msg::PoseStamped>
OmniPidPursuitController::softSmoothPathCorners(
  const std::vector<geometry_msgs::msg::PoseStamped> & path,
  double max_smooth_radius,
  int max_num_interpolation,
  double min_turn_angle_deg,
  double full_turn_angle_deg,
  double judge_window_dist,
  double min_point_spacing,
  int min_interpolation_points)
{
  if (path.size() < 3) {
    return path;
  }

  const double min_turn_angle =
    min_turn_angle_deg * M_PI / 180.0;

  const double full_turn_angle =
    full_turn_angle_deg * M_PI / 180.0;

  const double min_valid_len = 0.03;
  const double min_smooth_distance = 0.04;
  const double min_output_s_gap = 0.02;

  const unsigned char smooth_collision_cost_threshold =
    static_cast<unsigned char>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  std::vector<double> cum_s(path.size(), 0.0);

  for (size_t i = 1; i < path.size(); ++i) {
    const auto & p0 = path[i - 1].pose.position;
    const auto & p1 = path[i].pose.position;

    cum_s[i] =
      cum_s[i - 1] +
      std::hypot(p1.x - p0.x, p1.y - p0.y);
  }

  const double total_s = cum_s.back();

  if (total_s < 1e-4) {
    return path;
  }

  auto dist2d = [](
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  };

  auto make_pose = [&](
    const geometry_msgs::msg::PoseStamped & ref,
    double x,
    double y)
  {
    geometry_msgs::msg::PoseStamped p = ref;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;
    return p;
  };

  auto smoothstep = [](double x)
  {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  };

  auto pose_at_s = [&](double target_s)
  {
    target_s = std::clamp(target_s, 0.0, total_s);

    if (target_s <= 0.0) {
      return path.front();
    }

    if (target_s >= total_s) {
      return path.back();
    }

    auto it = std::lower_bound(cum_s.begin(), cum_s.end(), target_s);
    size_t idx = static_cast<size_t>(std::distance(cum_s.begin(), it));

    if (idx == 0) {
      return path.front();
    }

    const double s0 = cum_s[idx - 1];
    const double s1 = cum_s[idx];

    const double ratio =
      (target_s - s0) / std::max(1e-6, s1 - s0);

    const auto & p0 = path[idx - 1].pose.position;
    const auto & p1 = path[idx].pose.position;

    return make_pose(
      path[idx],
      p0.x + ratio * (p1.x - p0.x),
      p0.y + ratio * (p1.y - p0.y));
  };

  auto calc_interpolation_count = [&](
    double weight,
    double smooth_span)
  {
    if (weight < 0.05) {
      return 0;
    }

    const double min_required_span =
      static_cast<double>(min_interpolation_points + 1) *
      min_point_spacing;

    if (smooth_span < min_required_span) {
      return 0;
    }

    const double full_interp_span =
      std::max(0.35, 2.0 * max_smooth_radius);

    const double length_ratio =
      std::clamp(smooth_span / full_interp_span, 0.0, 1.0);

    const double score =
      static_cast<double>(max_num_interpolation) *
      (0.35 + 0.65 * weight) *
      length_ratio;

    int n_by_score =
      static_cast<int>(std::ceil(score));

    int n_by_spacing =
      static_cast<int>(std::floor(smooth_span / min_point_spacing)) - 1;

    n_by_score = std::max(0, n_by_score);
    n_by_spacing = std::max(0, n_by_spacing);

    int n = std::min({
      max_num_interpolation,
      n_by_score,
      n_by_spacing
    });

    if (n < min_interpolation_points) {
      return 0;
    }

    return n;
  };

  struct SmoothCandidate
  {
    size_t index{0};

    double si{0.0};
    double start_s{0.0};
    double end_s{0.0};
    double score{0.0};

    int sign{0};
    int n{0};

    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped control;
    geometry_msgs::msg::PoseStamped end;
  };

  auto make_candidate_points = [&](
    const SmoothCandidate & c,
    std::vector<geometry_msgs::msg::PoseStamped> & candidate,
    std::vector<double> & candidate_s)
  {
    candidate.clear();
    candidate_s.clear();

    const double smooth_span = c.end_s - c.start_s;

    candidate.reserve(c.n + 2);
    candidate_s.reserve(c.n + 2);

    candidate.push_back(c.start);
    candidate_s.push_back(c.start_s);

    const auto & p1 = c.control.pose.position;

    for (int k = 1; k <= c.n; ++k) {
      const double t =
        static_cast<double>(k) /
        static_cast<double>(c.n + 1);

      const double omt = 1.0 - t;

      const double x =
        omt * omt * c.start.pose.position.x +
        2.0 * omt * t * p1.x +
        t * t * c.end.pose.position.x;

      const double y =
        omt * omt * c.start.pose.position.y +
        2.0 * omt * t * p1.y +
        t * t * c.end.pose.position.y;

      candidate.push_back(make_pose(c.control, x, y));
      candidate_s.push_back(c.start_s + t * smooth_span);
    }

    candidate.push_back(c.end);
    candidate_s.push_back(c.end_s);
  };

  auto is_candidate_safe = [&](
    const SmoothCandidate & c)
  {
    std::vector<geometry_msgs::msg::PoseStamped> candidate;
    std::vector<double> candidate_s;

    make_candidate_points(c, candidate, candidate_s);

    if (candidate.size() < 2) {
      return false;
    }

    geometry_msgs::msg::PoseStamped last = candidate.front();

    for (size_t k = 1; k < candidate.size(); ++k) {
      if (checkLineCollisionWithThreshold(
            last,
            candidate[k],
            smooth_collision_cost_threshold))
      {
        return false;
      }

      last = candidate[k];
    }

    return true;
  };

  // ============================================================
  // 第一步：扫描所有“可插值候选点”
  // ============================================================
  std::vector<SmoothCandidate> candidates;
  candidates.reserve(path.size());

  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const double si = cum_s[i];

    const auto & curr = path[i];
    const auto & p1 = curr.pose.position;

    const double before_s =
      std::max(0.0, si - judge_window_dist);

    const double after_s =
      std::min(total_s, si + judge_window_dist);

    if (si - before_s < min_valid_len ||
        after_s - si < min_valid_len)
    {
      continue;
    }

    const auto before = pose_at_s(before_s);
    const auto after = pose_at_s(after_s);

    const auto & pb = before.pose.position;
    const auto & pa = after.pose.position;

    const double in_x = p1.x - pb.x;
    const double in_y = p1.y - pb.y;

    const double out_x = pa.x - p1.x;
    const double out_y = pa.y - p1.y;

    const double len_in = std::hypot(in_x, in_y);
    const double len_out = std::hypot(out_x, out_y);

    if (len_in < min_valid_len || len_out < min_valid_len) {
      continue;
    }

    const double uin_x = in_x / len_in;
    const double uin_y = in_y / len_in;

    const double uout_x = out_x / len_out;
    const double uout_y = out_y / len_out;

    double dot = uin_x * uout_x + uin_y * uout_y;
    dot = std::clamp(dot, -1.0, 1.0);

    const double turn_angle = std::acos(dot);

    if (turn_angle < min_turn_angle) {
      continue;
    }

    const double cross =
      uin_x * uout_y - uin_y * uout_x;

    const int sign = cross >= 0.0 ? 1 : -1;

    const double raw =
      (turn_angle - min_turn_angle) /
      std::max(1e-6, full_turn_angle - min_turn_angle);

    const double weight = smoothstep(raw);

    if (weight <= 1e-4) {
      continue;
    }

    double d =
      max_smooth_radius * (0.45 + 0.55 * weight);

    d = std::min({
      d,
      0.45 * (si - before_s),
      0.45 * (after_s - si),
      si - min_output_s_gap,
      total_s - si - min_output_s_gap
    });

    if (d < min_smooth_distance) {
      continue;
    }

    const double start_s = si - d;
    const double end_s = si + d;

    if (end_s <= start_s + min_output_s_gap) {
      continue;
    }

    const double smooth_span = end_s - start_s;

    const int n =
      calc_interpolation_count(weight, smooth_span);

    if (n < min_interpolation_points) {
      continue;
    }

    SmoothCandidate c;
    c.index = i;
    c.si = si;
    c.start_s = start_s;
    c.end_s = end_s;
    c.score = turn_angle * (0.5 + 0.5 * weight);
    c.sign = sign;
    c.n = n;
    c.start = pose_at_s(start_s);
    c.control = curr;
    c.end = pose_at_s(end_s);

    if (!is_candidate_safe(c)) {
      continue;
    }

    candidates.push_back(c);
  }

  if (candidates.empty()) {
    std::vector<LockedSmoothRegion> new_locks;
    new_locks.reserve(locked_smooth_regions_.size());

    for (auto lock : locked_smooth_regions_) {
      if (!lock.valid) {
        continue;
      }

      lock.miss_count++;

      if (lock.miss_count <= smooth_region_max_miss_) {
        new_locks.push_back(lock);
      }
    }

    locked_smooth_regions_.swap(new_locks);

    return path;
  }

  // ============================================================
  // 第二步：把连续候选点聚成“弯道区间”
  // sign 不同必须分开，避免 S 弯两个方向混成一个区间
  // ============================================================
  struct SmoothRegion
  {
    std::vector<size_t> candidate_indices;

    double start_s{0.0};
    double end_s{0.0};
    double center_s{0.0};

    int sign{0};

    geometry_msgs::msg::PoseStamped center;
  };

  std::vector<SmoothRegion> regions;

  const double region_group_gap =
    std::max(0.15, judge_window_dist * 1.2);

  for (size_t ci = 0; ci < candidates.size(); ++ci) {
    const auto & c = candidates[ci];

    const bool need_new_region =
      regions.empty() ||
      regions.back().sign != c.sign ||
      c.si - regions.back().end_s > region_group_gap;

    if (need_new_region) {
      SmoothRegion r;
      r.candidate_indices.push_back(ci);
      r.start_s = c.start_s;
      r.end_s = c.end_s;
      r.center_s = 0.5 * (r.start_s + r.end_s);
      r.sign = c.sign;
      r.center = pose_at_s(r.center_s);
      regions.push_back(r);
    } else {
      auto & r = regions.back();
      r.candidate_indices.push_back(ci);
      r.start_s = std::min(r.start_s, c.start_s);
      r.end_s = std::max(r.end_s, c.end_s);
      r.center_s = 0.5 * (r.start_s + r.end_s);
      r.center = pose_at_s(r.center_s);
    }
  }

  if (regions.empty()) {
    return path;
  }

// ============================================================
// 第三步：为每个弯道区间选择一个稳定的插值候选点
// ============================================================

  struct SelectedSmoothRegion
  {
    size_t region_idx{0};
    size_t candidate_idx{0};
    int matched_lock_idx{-1};
  };

  std::vector<SelectedSmoothRegion> selected_regions;
  selected_regions.reserve(regions.size());

  std::vector<bool> lock_used(locked_smooth_regions_.size(), false);

  for (size_t ri = 0; ri < regions.size(); ++ri) {
    const auto & r = regions[ri];

    if (selected_regions.size() >= static_cast<size_t>(smooth_region_max_count_)) {
      break;
    }

    int matched_lock_idx = -1;
    double best_lock_dist = std::numeric_limits<double>::infinity();

    // 1. 当前 region 尝试匹配旧锁定区间
    for (size_t li = 0; li < locked_smooth_regions_.size(); ++li) {
      const auto & lock = locked_smooth_regions_[li];

      if (!lock.valid) {
        continue;
      }

      if (li < lock_used.size() && lock_used[li]) {
        continue;
      }

      if (lock.sign != r.sign) {
        continue;
      }

      if (!lock.frame_id.empty() &&
          lock.frame_id != r.center.header.frame_id)
      {
        continue;
      }

      const double d = dist2d(lock.center, r.center);

      if (d < smooth_region_match_dist_ && d < best_lock_dist) {
        best_lock_dist = d;
        matched_lock_idx = static_cast<int>(li);
      }
    }

    // 2. 在当前 region 内选一个控制点
    size_t selected_candidate_idx = r.candidate_indices.front();

    if (matched_lock_idx >= 0) {
      // 如果这个区间上次已经锁定过，则优先选离上次 control 最近的候选点
      // 这样同一个弯内部不会 path[i] / path[i+1] 来回跳
      const auto & lock =
        locked_smooth_regions_[static_cast<size_t>(matched_lock_idx)];

      double best_control_dist = std::numeric_limits<double>::infinity();

      for (size_t ci : r.candidate_indices) {
        const double d = dist2d(candidates[ci].control, lock.control);

        if (d < best_control_dist) {
          best_control_dist = d;
          selected_candidate_idx = ci;
        }
      }

      lock_used[static_cast<size_t>(matched_lock_idx)] = true;
    } else {
      // 新区间：选 score 最大的点
      double best_score = -1.0;

      for (size_t ci : r.candidate_indices) {
        if (candidates[ci].score > best_score) {
          best_score = candidates[ci].score;
          selected_candidate_idx = ci;
        }
      }
    }

    SelectedSmoothRegion selected;
    selected.region_idx = ri;
    selected.candidate_idx = selected_candidate_idx;
    selected.matched_lock_idx = matched_lock_idx;

    selected_regions.push_back(selected);
  }

  if (selected_regions.empty()) {
    // 没有可插值区间，旧锁定区间计数递增
    std::vector<LockedSmoothRegion> new_locks;
    new_locks.reserve(locked_smooth_regions_.size());

    for (auto lock : locked_smooth_regions_) {
      if (!lock.valid) {
        continue;
      }

      lock.miss_count++;

      if (lock.miss_count <= smooth_region_max_miss_) {
        new_locks.push_back(lock);
      }
    }

    locked_smooth_regions_.swap(new_locks);

    return path;
  }

  // ============================================================
  // 第四步：输出路径
  // 多个 selected region 按 start_s 顺序依次插入
  // ============================================================

  std::vector<geometry_msgs::msg::PoseStamped> smoothed;
  smoothed.reserve(path.size() + selected_regions.size() * (max_num_interpolation + 2) + 4);

  double last_output_s = 0.0;

  auto append_no_duplicate = [&](
    const geometry_msgs::msg::PoseStamped & p)
  {
    if (!smoothed.empty() && dist2d(smoothed.back(), p) < 1e-4) {
      return false;
    }

    smoothed.push_back(p);
    return true;
  };

  auto append_at_s = [&](
    const geometry_msgs::msg::PoseStamped & p,
    double s)
  {
    if (!smoothed.empty() && s <= last_output_s + min_output_s_gap) {
      return false;
    }

    if (append_no_duplicate(p)) {
      last_output_s = s;
      return true;
    }

    return false;
  };

  smoothed.push_back(path.front());
  last_output_s = 0.0;

  size_t next_selected_idx = 0;

  for (size_t i = 1; i < path.size(); ++i) {
    const double si = cum_s[i];

    // 当前路径点已经被上一个平滑段覆盖
    if (si <= last_output_s + min_output_s_gap) {
      continue;
    }

    // 可能有多个平滑区间的 start_s 都已经到达
    while (next_selected_idx < selected_regions.size()) {
      const auto & selected_meta = selected_regions[next_selected_idx];
      const auto & selected =
        candidates[selected_meta.candidate_idx];

      if (si < selected.start_s) {
        break;
      }

      // 如果这个区间和上一个插值区间重叠太多，就跳过，避免插值段互相覆盖
      if (selected.start_s <= last_output_s + min_output_s_gap) {
        next_selected_idx++;
        continue;
      }

      std::vector<geometry_msgs::msg::PoseStamped> candidate;
      std::vector<double> candidate_s;

      make_candidate_points(selected, candidate, candidate_s);

      for (size_t k = 0; k < candidate.size(); ++k) {
        append_at_s(candidate[k], candidate_s[k]);
      }

      last_output_s =
        std::max(last_output_s, selected.end_s);

      next_selected_idx++;
    }

    if (si <= last_output_s + min_output_s_gap) {
      continue;
    }

    append_at_s(path[i], si);
  }

  const auto & last_p = path.back();

  if (smoothed.empty() || dist2d(smoothed.back(), last_p) > 1e-4) {
    smoothed.push_back(last_p);
  }

  // ============================================================
  // 第五步：更新多区间锁定缓存
  // selected region 变成新的有效锁；没有匹配到的旧锁 miss_count++
  // ============================================================

  std::vector<LockedSmoothRegion> new_locks;
  new_locks.reserve(selected_regions.size() + locked_smooth_regions_.size());

  // 1. 先写入本帧选中的所有 region
  for (const auto & selected_meta : selected_regions) {
    const auto & r =
      regions[selected_meta.region_idx];

    const auto & c =
      candidates[selected_meta.candidate_idx];

    LockedSmoothRegion lock;

    if (selected_meta.matched_lock_idx >= 0 &&
        static_cast<size_t>(selected_meta.matched_lock_idx) < locked_smooth_regions_.size())
    {
      lock =
        locked_smooth_regions_[static_cast<size_t>(selected_meta.matched_lock_idx)];
    }

    lock.valid = true;
    lock.frame_id = r.center.header.frame_id;
    lock.center = r.center;
    lock.control = c.control;
    lock.sign = r.sign;
    lock.miss_count = 0;

    new_locks.push_back(lock);
  }

  // 2. 没有匹配到的旧锁保留几帧，避免 region 短暂消失后立刻丢锁
  for (size_t li = 0; li < locked_smooth_regions_.size(); ++li) {
    if (li < lock_used.size() && lock_used[li]) {
      continue;
    }

    auto lock = locked_smooth_regions_[li];

    if (!lock.valid) {
      continue;
    }

    lock.miss_count++;

    if (lock.miss_count <= smooth_region_max_miss_) {
      new_locks.push_back(lock);
    }
  }

  locked_smooth_regions_.swap(new_locks);

  RCLCPP_DEBUG(
    logger_,
    "Smooth multi-region: regions=%zu, selected=%zu, locks=%zu",
    regions.size(),
    selected_regions.size(),
    locked_smooth_regions_.size());

  return smoothed;
  }
bool OmniPidPursuitController::checkLineCollisionWithThreshold(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  unsigned char cost_threshold)
{
  auto costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();

  geometry_msgs::msg::PoseStamped start_cm;
  geometry_msgs::msg::PoseStamped end_cm;

  if (!transformPose(costmap_frame, start, start_cm) ||
      !transformPose(costmap_frame, end, end_cm))
  {
    return true;
  }

  unsigned int mx0 = 0;
  unsigned int my0 = 0;
  unsigned int mx1 = 0;
  unsigned int my1 = 0;

  if (!costmap->worldToMap(start_cm.pose.position.x, start_cm.pose.position.y, mx0, my0) ||
      !costmap->worldToMap(end_cm.pose.position.x, end_cm.pose.position.y, mx1, my1))
  {
    return true;
  }

  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  int x0 = static_cast<int>(mx0);
  int y0 = static_cast<int>(my0);
  int x1 = static_cast<int>(mx1);
  int y1 = static_cast<int>(my1);

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);

  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;

  int err = dx - dy;
  int x = x0;
  int y = y0;

  const int max_iter = std::max(300, dx + dy + 10);
  int iter = 0;

  while (iter++ < max_iter) {
    if (x < 0 || x >= size_x || y < 0 || y >= size_y) {
      return true;
    }

    const unsigned char cost =
      costmap->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));

    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return true;
    }

    if (cost >= cost_threshold) {
      return true;
    }

    if (x == x1 && y == y1) {
      break;
    }

    const int e2 = 2 * err;

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
  return checkLineCollisionWithThreshold(start, end, 120);
}
std::vector<OmniMpcController::State>
OmniPidPursuitController::samplePathToTimedMpcRefSeq(
  const std::vector<geometry_msgs::msg::PoseStamped> & path_poses,
  int Np,
  double v_des,
  double * v_ref_eff_out) const
{
  std::vector<OmniMpcController::State> ref_seq;

  if (Np <= 0 || path_poses.empty()) {
    return ref_seq;
  }

  ref_seq.reserve(Np);

  if (path_poses.size() == 1) {
    OmniMpcController::State ref;
    ref.setZero();
    ref(0) = path_poses.front().pose.position.x;
    ref(1) = path_poses.front().pose.position.y;

    while (static_cast<int>(ref_seq.size()) < Np) {
      ref_seq.push_back(ref);
    }

    return ref_seq;
  }

  std::vector<double> cum_s(path_poses.size(), 0.0);

  for (size_t i = 1; i < path_poses.size(); ++i) {
    const auto & p0 = path_poses[i - 1].pose.position;
    const auto & p1 = path_poses[i].pose.position;

    cum_s[i] =
      cum_s[i - 1] +
      std::hypot(p1.x - p0.x, p1.y - p0.y);
  }

  const double total_s = cum_s.back();

  if (total_s < 1e-6) {
    OmniMpcController::State ref;
    ref.setZero();
    ref(0) = path_poses.back().pose.position.x;
    ref(1) = path_poses.back().pose.position.y;

    while (static_cast<int>(ref_seq.size()) < Np) {
      ref_seq.push_back(ref);
    }

    return ref_seq;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  double start_s = 0.0;

  const double start_s_deadband = 0.1;      
  const double projection_trust_dist = 0.50; 

  if (path_poses.size() >= 2) {
    const auto & p0 = path_poses[0].pose.position;
    const auto & p1 = path_poses[1].pose.position;

    const double x0 = p0.x;
    const double y0 = p0.y;
    const double x1 = p1.x;
    const double y1 = p1.y;

    const double dx = x1 - x0;
    const double dy = y1 - y0;

    const double len2 = dx * dx + dy * dy;

    if (len2 > 1e-10) {
      double t = -(x0 * dx + y0 * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);

      const double proj_x = x0 + t * dx;
      const double proj_y = y0 + t * dy;
      const double proj_dist = std::hypot(proj_x, proj_y);

      const double seg_len = std::sqrt(len2);
      double projected_start_s = t * seg_len;

      if (proj_dist > projection_trust_dist) {
        start_s = 0.0;
      }
      else if (projected_start_s < start_s_deadband) {
        start_s = 0.0;
      }
      else {
        start_s = projected_start_s;
      }
    }
  }

  start_s = std::clamp(start_s, 0.0, total_s);
  const double remaining_s = total_s - start_s;

  if (remaining_s < 1e-6) {
    OmniMpcController::State ref;
    ref.setZero();
    ref(0) = path_poses.back().pose.position.x;
    ref(1) = path_poses.back().pose.position.y;

    while (static_cast<int>(ref_seq.size()) < Np) {
      ref_seq.push_back(ref);
    }

    return ref_seq;
  }

  auto pose_at_s = [&](double target_s)
  {
    target_s = std::clamp(target_s, 0.0, total_s);

    auto it = std::lower_bound(cum_s.begin(), cum_s.end(), target_s);
    size_t idx = static_cast<size_t>(std::distance(cum_s.begin(), it));

    if (idx == 0) {
      return path_poses.front();
    }

    if (idx >= path_poses.size()) {
      return path_poses.back();
    }

    const double s0 = cum_s[idx - 1];
    const double s1 = cum_s[idx];

    const double ratio =
      (target_s - s0) / std::max(1e-6, s1 - s0);

    const auto & p0 = path_poses[idx - 1].pose.position;
    const auto & p1 = path_poses[idx].pose.position;

    geometry_msgs::msg::PoseStamped out = path_poses[idx];

    out.pose.position.x = p0.x + ratio * (p1.x - p0.x);
    out.pose.position.y = p0.y + ratio * (p1.y - p0.y);
    out.pose.position.z = 0.0;

    return out;
  };

  const double horizon_time =
    control_duration_ * static_cast<double>(Np);

  double v_ref_eff = 0.0;

  if (horizon_time > 1e-6) {
    v_ref_eff = std::min(v_des, remaining_s / horizon_time);
  }

  v_ref_eff = std::max(0.0, v_ref_eff);

  double max_kappa_ahead = 0.0;

  const double curve_check_dist =
    std::clamp(
      mpc_curve_lookahead_dist_,
      0.2,
      std::max(0.2, remaining_s));

  const double ds = 0.10;

  for (double s = start_s; s <= std::min(total_s, start_s + curve_check_dist); s += ds) {
    const double s_back = std::max(0.0, s - curvature_backward_dist_);
    const double s_mid = std::clamp(s, 0.0, total_s);
    const double s_fwd = std::min(total_s, s + curvature_forward_dist_);

    if (s_fwd - s_back < 0.05) {
      continue;
    }

    const auto p_back = pose_at_s(s_back);
    const auto p_mid = pose_at_s(s_mid);
    const auto p_fwd = pose_at_s(s_fwd);

    const auto & a = p_back.pose.position;
    const auto & b = p_mid.pose.position;
    const auto & c = p_fwd.pose.position;

    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ca = std::hypot(a.x - c.x, a.y - c.y);

    if (ab < 1e-4 || bc < 1e-4 || ca < 1e-4) {
      continue;
    }

    // 三点曲率：
    // kappa = 4 * triangle_area / (ab * bc * ca)
    const double cross =
      std::abs(
        (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x));

    const double kappa =
      2.0 * cross / std::max(1e-9, ab * bc * ca);

    if (std::isfinite(kappa)) {
      max_kappa_ahead = std::max(max_kappa_ahead, kappa);
    }
  }

  double v_curve_limit = v_des;

  if (max_kappa_ahead > mpc_curve_kappa_eps_) {
    v_curve_limit =
      std::sqrt(
        std::max(0.0, mpc_curve_a_lat_max_) /
        std::max(max_kappa_ahead, mpc_curve_kappa_eps_));
  }

  v_curve_limit =
    std::clamp(
      v_curve_limit,
      std::max(0.0, mpc_curve_min_speed_),
      v_des);

  v_ref_eff = std::min(v_ref_eff, v_curve_limit);

  if (v_ref_eff_out != nullptr) {
    *v_ref_eff_out = v_ref_eff;
  }
  auto estimate_tangent_by_center_diff =
    [&](double target_s, Eigen::Vector2d & tangent) -> bool
  {
    tangent.setZero();

    const double ref_step_s =
      std::max(v_ref_eff * control_duration_, 0.05);

    const double tangent_sample_dist =
      std::clamp(2.0 * ref_step_s, 0.12, 0.30);

    double s_back = std::max(0.0, target_s - tangent_sample_dist);
    double s_fwd = std::min(total_s, target_s + tangent_sample_dist);

    if (s_fwd - s_back < 1e-4) {
      return false;
    }

    const auto p_back = pose_at_s(s_back);
    const auto p_fwd = pose_at_s(s_fwd);

    const auto & pb = p_back.pose.position;
    const auto & pf = p_fwd.pose.position;

    const double dx = pf.x - pb.x;
    const double dy = pf.y - pb.y;

    const double norm = std::hypot(dx, dy);

    if (norm < 1e-6 || !std::isfinite(norm)) {
      return false;
    }

    tangent.x() = dx / norm;
    tangent.y() = dy / norm;
    return true;
  };

  auto estimate_tangent_by_local_poly =
    [&](double target_s, Eigen::Vector2d & tangent) -> bool
  {
    tangent.setZero();

    // 用 5 个等弧长采样点做三次局部拟合。
    // 注意：这里只用多项式求导，不改变 ref(0), ref(1) 的位置参考。
    const int sample_count = 5;
    const int max_order = 3;

    auto it = std::lower_bound(cum_s.begin(), cum_s.end(), target_s);
    size_t idx = static_cast<size_t>(std::distance(cum_s.begin(), it));

    if (idx >= cum_s.size()) {
      idx = cum_s.size() - 1;
    }

    const size_t left_idx =
      idx > 3 ? idx - 3 : 0;

    const size_t right_idx =
      std::min(cum_s.size() - 1, idx + 3);

    double spacing_sum = 0.0;
    int spacing_count = 0;

    for (size_t k = left_idx + 1; k <= right_idx; ++k) {
      const double seg_len = cum_s[k] - cum_s[k - 1];

      if (seg_len > 1e-4 && std::isfinite(seg_len)) {
        spacing_sum += seg_len;
        spacing_count++;
      }
    }

    const double local_spacing =
      spacing_count > 0 ?
      spacing_sum / static_cast<double>(spacing_count) :
      0.10;

    const double ref_step_s =
      std::max(v_ref_eff * control_duration_, 0.05);

    double half_window =
      std::max({
        2.0 * ref_step_s,
        3.0 * local_spacing,
        0.12
      });

    half_window = std::clamp(half_window, 0.12, 0.35);

    half_window = std::min(half_window, 0.5 * total_s);

    if (half_window < 1e-3) {
      return false;
    }

    double left_s = target_s - half_window;
    double right_s = target_s + half_window;

    if (left_s < 0.0) {
      right_s = std::min(total_s, right_s - left_s);
      left_s = 0.0;
    }

    if (right_s > total_s) {
      left_s = std::max(0.0, left_s - (right_s - total_s));
      right_s = total_s;
    }

    if (right_s - left_s < 0.08) {
      return false;
    }

    struct PolySample
    {
      double s{0.0};
      double x{0.0};
      double y{0.0};
    };

    std::vector<PolySample> samples;
    samples.reserve(sample_count);

    for (int k = 0; k < sample_count; ++k) {
      const double ratio =
        static_cast<double>(k) /
        static_cast<double>(sample_count - 1);

      const double s =
        left_s + ratio * (right_s - left_s);

      const auto p = pose_at_s(s);
      const auto & pp = p.pose.position;

      if (!std::isfinite(pp.x) || !std::isfinite(pp.y)) {
        continue;
      }

      PolySample sample;
      sample.s = s;
      sample.x = pp.x;
      sample.y = pp.y;
      samples.push_back(sample);
    }

    if (samples.size() < 2) {
      return false;
    }

    const int order =
      std::min<int>(
        max_order,
        static_cast<int>(samples.size()) - 1);

    if (order < 1) {
      return false;
    }

    Eigen::MatrixXd A(samples.size(), order + 1);
    Eigen::VectorXd bx(samples.size());
    Eigen::VectorXd by(samples.size());

    for (size_t r = 0; r < samples.size(); ++r) {
      const double u =
        (samples[r].s - target_s) / std::max(half_window, 1e-6);

      double uk = 1.0;
      for (int c = 0; c <= order; ++c) {
        A(static_cast<int>(r), c) = uk;
        uk *= u;
      }

      bx(static_cast<int>(r)) = samples[r].x;
      by(static_cast<int>(r)) = samples[r].y;
    }

    const auto qr = A.colPivHouseholderQr();

    const Eigen::VectorXd coeff_x = qr.solve(bx);
    const Eigen::VectorXd coeff_y = qr.solve(by);

    if (coeff_x.size() < 2 || coeff_y.size() < 2) {
      return false;
    }

    const double dx_ds =
      coeff_x(1) / std::max(half_window, 1e-6);

    const double dy_ds =
      coeff_y(1) / std::max(half_window, 1e-6);

    const double norm = std::hypot(dx_ds, dy_ds);

    if (norm < 1e-6 || !std::isfinite(norm)) {
      return false;
    }

    tangent.x() = dx_ds / norm;
    tangent.y() = dy_ds / norm;

    return true;
  };
  std::vector<double> ref_s;
  std::vector<Eigen::Vector2d> ref_pos;

  ref_s.reserve(Np);
  ref_pos.reserve(Np);

  for (int i = 1; i <= Np; ++i) {
    const double target_s =
      std::min(
        total_s,
        start_s + v_ref_eff * control_duration_ * static_cast<double>(i));

    const auto sampled_pose = pose_at_s(target_s);

    ref_s.push_back(target_s);
    ref_pos.emplace_back(
      sampled_pose.pose.position.x,
      sampled_pose.pose.position.y);
  }

  auto normalize_vec = [](
    const Eigen::Vector2d & v,
    Eigen::Vector2d & out) -> bool
  {
    const double n = v.norm();

    if (n < 1e-6 || !std::isfinite(n)) {
      out.setZero();
      return false;
    }

    out = v / n;
    return true;
  };

  auto estimate_tangent_by_ref_pos_diff =
    [&](int i, Eigen::Vector2d & tangent) -> bool
  {
    tangent.setZero();

    if (ref_pos.empty()) {
      return false;
    }

    Eigen::Vector2d d;
    d.setZero();

    if (i == 0) {
      // 第一个点：直接用机器人原点 -> 第一个参考点方向
      // 因为 path 已经在 base_link 下，机器人就是 (0,0)
      d = ref_pos[0];

      if (d.norm() < 1e-4 && ref_pos.size() >= 2) {
        d = ref_pos[1] - ref_pos[0];
      }
    } else if (i + 1 < static_cast<int>(ref_pos.size())) {
      // 后面的点：用中心差分
      d = ref_pos[i + 1] - ref_pos[i - 1];
    } else if (i > 0) {
      // 最后一个点：用后向差分
      d = ref_pos[i] - ref_pos[i - 1];
    }

    return normalize_vec(d, tangent);
  };

  const double max_poly_weight = 0.5;  // 先小一点，避免多项式导数滞后拖尾
  const double min_poly_weight = 0.0;

  for (int i = 0; i < Np; ++i) {
    OmniMpcController::State ref;
    ref.setZero();

    ref(0) = ref_pos[i].x();
    ref(1) = ref_pos[i].y();

    double vx_ref = 0.0;
    double vy_ref = 0.0;

    Eigen::Vector2d diff_dir;
    Eigen::Vector2d poly_dir;
    Eigen::Vector2d dir;

    diff_dir.setZero();
    poly_dir.setZero();
    dir.setZero();

    const bool diff_ok =
      estimate_tangent_by_ref_pos_diff(i, diff_dir);

    const bool poly_ok =
      estimate_tangent_by_local_poly(ref_s[i], poly_dir);

    if (i == 0) {
      // 第一个点必须快速响应当前路径，不用多项式
      if (diff_ok) {
        dir = diff_dir;
      }
    } else {
      if (diff_ok && poly_ok) {
        // 越靠后的预测点，多项式权重越大
        const double horizon_ratio =
          static_cast<double>(i) /
          std::max(1.0, static_cast<double>(Np - 1));

        const double poly_weight =
          std::clamp(
            min_poly_weight +
            (max_poly_weight - min_poly_weight) * horizon_ratio,
            min_poly_weight,
            max_poly_weight);

        dir =
          (1.0 - poly_weight) * diff_dir +
          poly_weight * poly_dir;
      } else if (diff_ok) {
        dir = diff_dir;
      } else if (poly_ok) {
        dir = poly_dir;
      }
    }

    const double dir_norm = dir.norm();

    if (dir_norm > 1e-6 && std::isfinite(dir_norm)) {
      dir /= dir_norm;

      vx_ref = v_ref_eff * dir.x();
      vy_ref = v_ref_eff * dir.y();
    }

    if (!std::isfinite(vx_ref)) {
      vx_ref = 0.0;
    }

    if (!std::isfinite(vy_ref)) {
      vy_ref = 0.0;
    }

    const double v_norm = std::hypot(vx_ref, vy_ref);

    if (v_norm > v_ref_eff && v_norm > 1e-6) {
      const double scale = v_ref_eff / v_norm;
      vx_ref *= scale;
      vy_ref *= scale;
    }

    ref(2) = vx_ref;
    ref(3) = vy_ref;

    ref_seq.push_back(ref);
  }
  std::cout << "total_s: " << total_s
          << " start_s: " << start_s
          << " remaining_s: " << remaining_s
          << " v_des: " << v_des
          << " v_ref_eff: " << v_ref_eff
          << std::endl;
  return ref_seq;
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
  if (original_linear_vel < 1e-6) {
    return;
  }
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
      else if (name == plugin_name_ + ".mpc_S_x") {
        mpc_S_x = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_S_y") {
        mpc_S_y = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_S_vx") {
        mpc_S_vx = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_S_vy") {
        mpc_S_vy = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_Q_x") {
        mpc_Q_x = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_Q_y") {
        mpc_Q_y = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_Q_vx") {
        mpc_Q_vx = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_Q_vy") {
        mpc_Q_vy = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_R_vx") {
        mpc_R_vx = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_R_vy") {
        mpc_R_vy = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_Rdelta_vx") {
        mpc_Rdelta_vx = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
        }
      } else if (name == plugin_name_ + ".mpc_Rdelta_vy") {
        mpc_Rdelta_vy = parameter.as_double();
        if (use_mpc_control_ && mpc_controller_) {
          updateMpcWeights();
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
        mpc_controller_->setHorizon(mpc_Np_, mpc_Nc_);
      }
    } else if (name == plugin_name_ + ".mpc_Nc") {
      mpc_Nc_ = parameter.as_int();
      if (use_mpc_control_ && mpc_controller_) {
        mpc_controller_->setHorizon(mpc_Np_, mpc_Nc_);
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
      -acc_max_,
      acc_max_,
      v_linear_min_,
      v_linear_max_);
      // 重新设置MPC权重
    updateMpcWeights();
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