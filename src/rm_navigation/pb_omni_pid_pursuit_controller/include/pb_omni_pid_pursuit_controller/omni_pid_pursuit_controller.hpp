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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include "nav2_core/controller.hpp"
#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "pb_omni_pid_pursuit_controller/mpc.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <atomic>

namespace pb_omni_pid_pursuit_controller
{
  
// Savitzky-Golay滤波实现
class SavitzkyGolayFilter
{
public:
  SavitzkyGolayFilter(int window_size = 5, int poly_order = 2)
    : window_size_(window_size), poly_order_(poly_order)
  {
    // 窗口大小必须是奇数，且大于多项式阶数
    if (window_size % 2 == 0) {
      window_size_ += 1;
    }
    if (window_size_ <= poly_order_) {
      poly_order_ = window_size_ - 1;
    }
    // 预计算滤波系数
    calculateCoefficients();
  }

  // 设置窗口大小（需重新计算系数）
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

  // 设置多项式阶数（需重新计算系数）
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
    // 1. 纯FIFO滑动窗口：最新值添加到窗口末尾
    data_window_.push_back(new_value);
    // std::cout << "size" << data_window_.size() << std::endl;
    // 2. 窗口超过设定大小 → 删除最旧的第一个值（保证窗口长度=window_size_）
    if (data_window_.size() > window_size_) {
      data_window_.erase(data_window_.begin());
    }

    // 3. 窗口未满（长度<设定大小）→ 直接返回原始值
    if (data_window_.size() < window_size_) {
      return new_value;
    }

    // 4. 窗口已满 → 计算“最新值（末尾）”的SG滤波值
    Eigen::Map<const Eigen::VectorXd> data_map(data_window_.data(), window_size_);
    double filtered_value = (window_weights_.transpose() * data_map)(0, 0);

    // 打印5维系数（验证）
    // std::cout << "5维窗口系数：";
    // for (int i = 0; i < window_weights_.size(); ++i) {
    //   std::cout << window_weights_(i) << " ";
    // }
    // std::cout << std::endl;
    return filtered_value;
  }

  // 重置滤波窗口
  void reset()
  {
    std::fill(data_window_.begin(), data_window_.end(), 0.0); // 窗口值置0
    valid_data_count_ = 0;                                   // 有效数据清零
  }
private:
  // 计算SG滤波系数
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

    // 计算系数：(V^T V)^{-1} V^T * e_m（e_m是中间位置的单位向量）
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

  int window_size_;          // 滤波窗口大小（奇数）
  int poly_order_;           // 多项式拟合阶数
  std::vector<double> coeffs_;  // 滤波系数
  Eigen::VectorXd window_weights_;
  Eigen::VectorXd coeffs;
  std::vector<double> data_window_;  // 数据滑动窗口
  int valid_data_count_ = 0;  
  int window_idx_; 
  
};
/**
 * @class pb_omni_pid_pursuit_controller::OmniPidPursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for
   * pb_omni_pid_pursuit_controller::OmniPidPursuitController
   */
  OmniPidPursuitController();

  /**
   * @brief Destrructor for
   * pb_omni_pid_pursuit_controller::OmniPidPursuitController
   */
  ~OmniPidPursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful
   * in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses
   * ineligible for lookaheadPoint Points ineligible to be selected as a
   * lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  std::vector<Eigen::Vector2d> samplePathToRefSeq(
    const std::vector<geometry_msgs::msg::PoseStamped> &path_poses,
    int Np);
  /**
   * @brief Gets the maximum extent of the costmap
   * @return Maximum costmap extent in meters
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Creates a Carrot Point Marker message for visualization
   * @param carrot_pose Lookahead point pose
   * @return Unique pointer to the Carrot Point Marker message
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Gets the lookahead point on the transformed plan
   * @param lookahead_dist Lookahead distance
   * @param transformed_plan Transformed local plan
   * @return Lookahead point pose
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief Calculates the intersection point of a circle and a line segment
   * @param p1 Start point of the line segment
   * @param p2 End point of the line segment
   * @param r Radius of the circle
   * @return Intersection point (geometry_msgs::msg::Point)
   */
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  /**
   * @brief Callback function for dynamic parameter updates
   * @param parameters Vector of updated parameters
   * @return Result of parameter setting
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Calculates the lookahead distance based on current velocity
   * @param speed Current robot velocity
   * @return Lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  /**
   * @brief Calculates the approach velocity scaling factor based on remaining path distance
   * @param path Transformed local path
   * @return Velocity scaling factor
   */
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Applies velocity scaling based on approach distance to the goal
   * @param path Transformed local path
   * @param linear_vel Linear velocity command (in out)
   */
  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;

  /**
   * @brief Checks if collision is detected along the given path
   * @param path Local path to check for collisions
   * @return True if collision detected, false otherwise
   */
  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
    struct Impl
    {
      // SG滤波实例
      std::unique_ptr<SavitzkyGolayFilter> linear_vel_filter_;
      std::unique_ptr<SavitzkyGolayFilter> angular_vel_filter_;
      // 滤波参数
      int sg_window_size_ = 5;
      int sg_poly_order_ = 2;
      bool enable_sg_filter_ = true;
    };

    // 4. 新增：impl_成员变量（智能指针）
    std::unique_ptr<Impl> impl_;
  /**
   * @brief Applies curvature based speed limitation
   * @param path Transformed local path
   * @param lookahead_pose Lookahead point pose
   * @param linear_vel Linear velocity command (in out)
   */
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);
  bool isDirectPathSafeToTarget(
    const geometry_msgs::msg::PoseStamped & target_pose) const;
  /**
   * @brief Calculates curvature using three-point circle fitting
   * @param path Transformed local path
   * @param lookahead_pose Lookahead pose (current point)
   * @param forward_dist
   * @param backward_dist
   * @return Curvature value
   */
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  /**
   * @brief Calculates the radius of curvature using three points
   * @param near_point Pose before the current point
   * @param current_point Current pose (lookahead pose)
   * @param far_point Pose after the current point
   * @return Radius of curvature
   */
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  /**
   * @brief Visualizes near and far points used for curvature calculation
   * @param backward_pose Near point pose
   * @param forward_pose Far point pose
   */
  void applyCurvatureLimitation_mpc(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & vx,double & vy);
  void applyApproachVelocityScaling_mpc(
    const nav_msgs::msg::Path & path, double &vx, double &vy);
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  /**
   * @brief Calculates cumulative distances along the path
   * @param path The path to calculate distances for
   * @return Vector of cumulative distances
   */
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Finds a pose on the path at a given distance
   * @param path The path to search on
   * @param cumulative_distances Vector of cumulative distances along the path
   * @param target_distance The target distance to find the pose at
   * @return Pose at the target distance, or empty pose if not found
   */
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;
  void smoothedVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) ;
  nav_msgs::msg::Path cropGlobalPlanToLocal(
    const geometry_msgs::msg::PoseStamped & pose);
  std::vector<geometry_msgs::msg::PoseStamped> removeCornerPts(
    const std::vector<geometry_msgs::msg::PoseStamped> &path);
  double euclideanDistance(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2);
  bool checkLineCollision(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end);
  Eigen::Vector2d poseToEigen(
    const geometry_msgs::msg::PoseStamped & pose)
  {
      return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
  }
  std::vector<geometry_msgs::msg::PoseStamped> smoothPathCorners(
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    double smooth_radius ,int num_interpolation,
    double angle_tol_deg ,int skip_points
  );
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  double last_velocity_scaling_factor_;
  bool has_prev_cmd_vel_ = false;
  
  geometry_msgs::msg::TwistStamped prev_cmd_vel_;

  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> heading_pid_;
  std::unique_ptr<OmniMpcController> mpc_controller_;
  int mpc_Np_;         // MPC预测时域
  int mpc_Nc_;         // MPC控制时域
  double mpc_Q_x,mpc_Q_y,mpc_R_vx,mpc_R_vy,mpc_Rdelta_vx,mpc_Rdelta_vy;
  // Controller parameters
  double translation_kp_, translation_ki_, translation_kd_;
  bool enable_rotation_;
  double rotation_kp_, rotation_ki_, rotation_kd_;
  double min_max_sum_error_;
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
  double large_slow_;
  double acc_max_;
  tf2::Duration transform_tolerance_;
  double smoothed_vel;
  bool slow = false;
  std::mutex sm_mutex;
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;
  // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  // std::unique_ptr<minco_nav2::MincoTracker> minco_tracker_;
  const double max_skip_distance = 4.0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rm_task_sub_;
  std::atomic<int> rm_task_value_{0};

  double direct_drive_kp_ = 1.5;
  double direct_drive_max_vel_ = 3.0;
  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smooth_vel_sub_;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
