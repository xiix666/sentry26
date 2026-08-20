#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "nav2_core/controller.hpp"
#include "rps_omni_controller/pid.hpp"
#include "rps_omni_controller/mpc.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <atomic>
#include <Eigen/QR>
#include <chrono>
#include <cstdint>
namespace rps_omni_controller
{

class SavitzkyGolayFilter
{
public:
  SavitzkyGolayFilter(int window_size = 5, int poly_order = 2)
    : window_size_(window_size), poly_order_(poly_order)
  {

    if (window_size % 2 == 0) {
      window_size_ += 1;
    }
    if (window_size_ <= poly_order_) {
      poly_order_ = window_size_ - 1;
    }

    calculateCoefficients();
  }

  void setWindowSize(int window_size)
  {
    if (window_size % 2 == 0) {
      window_size += 1;
    }
    window_size_ = window_size;
    if (window_size_ <= poly_order_) {
      poly_order_ = window_size_ - 1;
    }
    calculateCoefficients();
  }

  void setPolyOrder(int poly_order)
  {
    if (window_size_ <= poly_order) {
      poly_order = window_size_ - 1;
    }
    poly_order_ = poly_order;
    calculateCoefficients();
  }

  double filter(double new_value)
  {

    data_window_.push_back(new_value);

    if (data_window_.size() > window_size_) {
      data_window_.erase(data_window_.begin());
    }

    if (data_window_.size() < window_size_) {
      return new_value;
    }

    Eigen::Map<const Eigen::VectorXd> data_map(data_window_.data(), window_size_);
    double filtered_value = (window_weights_.transpose() * data_map)(0, 0);

    return filtered_value;
  }

  void reset()
  {
    std::fill(data_window_.begin(), data_window_.end(), 0.0);
  }
private:

  void calculateCoefficients()
  {
    int m = window_size_ / 2;
    int n = poly_order_;
    std::vector<double> x_list;

    Eigen::MatrixXd V(window_size_, n + 1);
    for (int i = 0; i < window_size_; ++i) {
      int x = i - m;
      x_list.push_back(x);
      for (int j = 0; j <= n; ++j) {
        V(i, j) = pow(x, j);
      }
    }

    Eigen::VectorXd e_last = Eigen::VectorXd::Zero(window_size_);
    e_last(window_size_-1) = 1.0;
    coeffs = (V.transpose() * V).inverse() * V.transpose() * e_last;

    Eigen::VectorXd weights(window_size_);
    double x_last = x_list.back(); 
    for (int i = 0; i < window_size_; ++i) {
      weights(i) = 0.0;
      for (int j = 0; j <= n; ++j) {
        weights(i) += coeffs(j) * pow(x_list[i], j);
      }
    }
    window_weights_ = weights;
  }

  int window_size_;
  int poly_order_;
  Eigen::VectorXd window_weights_;
  Eigen::VectorXd coeffs;
  std::vector<double> data_window_;

};
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  OmniPidPursuitController();

  ~OmniPidPursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  std::vector<OmniMpcController::State> samplePathToTimedMpcRefSeq(
    const std::vector<geometry_msgs::msg::PoseStamped> & path_poses,
    int Np,
    double v_des,
    double * v_ref_eff_out = nullptr) const;
  void updateMpcWeights();
  double getCostmapMaxExtent() const;

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;

private:
    struct Impl
    {

      std::unique_ptr<SavitzkyGolayFilter> linear_vel_filter_;
      int sg_window_size_ = 5;
      int sg_poly_order_ = 2;
      bool enable_sg_filter_ = true;
    };

    std::unique_ptr<Impl> impl_;
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);
  bool isDirectPathSafeToTarget(
    const geometry_msgs::msg::PoseStamped & target_pose) const;
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  void applyCurvatureLimitation_mpc(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & vx,double & vy);
  void applyApproachVelocityScaling_mpc(
    const nav_msgs::msg::Path & path, double &vx, double &vy);
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;
  std::vector<geometry_msgs::msg::PoseStamped> removeCornerPts(
    const std::vector<geometry_msgs::msg::PoseStamped> &path);
  double euclideanDistance(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2);
  bool checkLineCollision(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end);
  bool checkLineCollisionWithThreshold(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    unsigned char cost_threshold);
  std::vector<geometry_msgs::msg::PoseStamped> softSmoothPathCorners(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double max_smooth_radius,
    int max_num_interpolation,
    double min_turn_angle_deg,
    double full_turn_angle_deg,
    double judge_window_dist,
    double min_point_spacing,
    int min_interpolation_points);
  struct LockedSmoothRegion
  {
    bool valid{false};

    std::string frame_id;

    geometry_msgs::msg::PoseStamped center;
    geometry_msgs::msg::PoseStamped control;

    int sign{0};
    int miss_count{0};
  };

  std::vector<LockedSmoothRegion> locked_smooth_regions_;

  double smooth_region_match_dist_{0.3};
  int smooth_region_max_miss_{5};
  int smooth_region_max_count_{5};   
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  double last_velocity_scaling_factor_;
  bool has_prev_cmd_vel_ = false;
  bool mpc_near_goal_pursuit_mode_{false};
  geometry_msgs::msg::TwistStamped prev_cmd_vel_;

  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> heading_pid_;
  std::unique_ptr<OmniMpcController> mpc_controller_;
  int mpc_Np_;
  int mpc_Nc_;
  double mpc_ref_speed_{2.5};
  double mpc_S_x{15.0};
  double mpc_S_y{15.0};
  double mpc_S_vx{0.0};
  double mpc_S_vy{0.0};

  double mpc_Q_x{5.0};
  double mpc_Q_y{5.0};
  double mpc_Q_vx{0.5};
  double mpc_Q_vy{0.5};

  double mpc_R_vx{0.5};
  double mpc_R_vy{0.5};
  double mpc_Rdelta_vx{0.5};
  double mpc_Rdelta_vy{0.5};

  double mpc_curve_a_lat_max_{1.0};
  double mpc_curve_min_speed_{1.0};
  double mpc_curve_lookahead_dist_{1.2};
  double mpc_curve_kappa_eps_{1e-3};

  double pid_curve_kappa_min_{0.35};
  double pid_curve_kappa_max_{1.20};
  double pid_curve_a_lat_max_{0.80};
  double pid_curve_min_speed_{0.35};
  double pid_curve_max_slowdown_ratio_{0.70};
  double pid_curve_lookahead_dist_{1.20};
  double pid_curve_sample_ds_{0.10};
  double pid_curve_projection_search_dist_{1.5};
  double pid_curve_projection_trust_dist_{0.50};
  double pid_curve_recover_rate_{1.0};
  double pid_curve_kappa_hysteresis_{0.08};
  bool pid_curve_slow_active_{false};
  double pid_curve_filtered_kappa_{0.0};
  bool has_pid_curve_filtered_kappa_{false};

  double pid_curve_kappa_rise_rate_{3.0};
  double pid_curve_kappa_fall_rate_{1.0};

  double translation_kp_, translation_ki_, translation_kd_;
  bool enable_rotation_;
  double rotation_kp_, rotation_ki_, rotation_kd_;
  double control_duration_;
  double max_robot_pose_search_dist_;
  bool use_interpolation_,use_mpc_control_;
  double lookahead_dist_;
  bool use_velocity_scaled_lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_rotate_to_heading_;
  double use_rotate_to_heading_treshold_;
  double v_linear_min_;
  double v_linear_max_;
  double v_angular_min_;
  double v_angular_max_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double max_velocity_scaling_factor_rate_;
  double last_vel_;
  double lower_speed_;
  double min_dist_;
  double acc_max_;
  mutable double mpc_curve_filtered_kappa_{0.0};
  mutable bool has_mpc_curve_filtered_kappa_{false};

  double mpc_curve_kappa_rise_rate_{3.0};
  double mpc_curve_kappa_fall_rate_{1.0};
  
  tf2::Duration transform_tolerance_;
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;

  const double max_skip_distance = 4.0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rm_task_sub_;

  std::atomic<int32_t> rm_task_value_{0};

  std::atomic<int64_t> last_rm_task_receive_time_ns_{0};

  double rm_task_timeout_sec_{0.5};

  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  bool has_latched_goal_{false};
  geometry_msgs::msg::PoseStamped latched_goal_;
  bool pending_controller_soft_reset_{false};

};

}

#endif
