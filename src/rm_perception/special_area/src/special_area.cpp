#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "nav2_msgs/srv/get_costmap.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include <cmath>
#include <limits>
#include <atomic>
class SpecialArea : public rclcpp::Node {
public:
  SpecialArea(const rclcpp::NodeOptions &node_options)
      : Node("special_area", node_options) {
    RCLCPP_INFO(this->get_logger(), "SpecialArea node has been started.");

    this->declare_parameter("left_top_x", 0.3);
    this->declare_parameter("left_top_y", -5.620203);
    this->declare_parameter("right_bottom_x", 5.6);
    this->declare_parameter("right_bottom_y", -8.035625);

    this->declare_parameter("service_name", "/global_costmap/get_costmap");
    this->get_parameter("left_top_x", left_top_x);
    this->get_parameter("left_top_y", left_top_y);
    this->get_parameter("right_bottom_x", right_bottom_x);
    this->get_parameter("right_bottom_y", right_bottom_y);
    this->get_parameter("service_name", service_name_);

    this->declare_parameter("third_p1_x", 17.02);
    this->declare_parameter("third_p1_y", 5.991264);
    this->declare_parameter("third_p2_x", 19.898225);
    this->declare_parameter("third_p2_y", 7.1);
    this->declare_parameter("third_hysteresis_margin", 0.2);

    this->get_parameter("third_p1_x", third_p1_x_);
    this->get_parameter("third_p1_y", third_p1_y_);
    this->get_parameter("third_p2_x", third_p2_x_);
    this->get_parameter("third_p2_y", third_p2_y_);
    this->get_parameter("third_hysteresis_margin", third_hysteresis_margin_);

    this->declare_parameter("special_p1_x", 8.4);
    this->declare_parameter("special_p1_y", 5.2);
    this->declare_parameter("special_p2_x", 5.8);
    this->declare_parameter("special_p2_y", 1.8);
    this->declare_parameter("special_p3_x", 4.6);
    this->declare_parameter("special_p3_y", 1.8);
    this->declare_parameter("special_p4_x", 7.2);
    this->declare_parameter("special_p4_y", 5.2);
    this->declare_parameter("special_hysteresis_margin", 0.2);
    this->declare_parameter("special_active_duration_sec", 6.0);

    this->get_parameter("special_p1_x", special_p1_x_);
    this->get_parameter("special_p1_y", special_p1_y_);
    this->get_parameter("special_p2_x", special_p2_x_);
    this->get_parameter("special_p2_y", special_p2_y_);
    this->get_parameter("special_p3_x", special_p3_x_);
    this->get_parameter("special_p3_y", special_p3_y_);
    this->get_parameter("special_p4_x", special_p4_x_);
    this->get_parameter("special_p4_y", special_p4_y_);
    this->get_parameter("special_hysteresis_margin",
                        special_hysteresis_margin_);
    this->get_parameter("special_active_duration_sec",
                        special_active_duration_sec_);

    fight_outpost_on_island_ =
        this->declare_parameter<bool>("fight_outpost_on_island", false);

    special_polygon_[0] = {special_p1_x_, special_p1_y_};
    special_polygon_[1] = {special_p2_x_, special_p2_y_};
    special_polygon_[2] = {special_p3_x_, special_p3_y_};
    special_polygon_[3] = {special_p4_x_, special_p4_y_};

    special_hysteresis_polygon_ =
        offsetConvexPolygon(special_polygon_, special_hysteresis_margin_);

    min_x = std::min(left_top_x, right_bottom_x);
    max_x = std::max(left_top_x, right_bottom_x);
    min_y = std::min(left_top_y, right_bottom_y);
    max_y = std::max(left_top_y, right_bottom_y);

    hysteresis_min_x = min_x - 0.2;
    hysteresis_max_x = max_x + 0.2;
    hysteresis_min_y = min_y - 0.2;
    hysteresis_max_y = max_y + 0.2;

    third_min_x_ = std::min(third_p1_x_, third_p2_x_);
    third_max_x_ = std::max(third_p1_x_, third_p2_x_);
    third_min_y_ = std::min(third_p1_y_, third_p2_y_);
    third_max_y_ = std::max(third_p1_y_, third_p2_y_);

    third_hysteresis_min_x_ = third_min_x_ - third_hysteresis_margin_;
    third_hysteresis_max_x_ = third_max_x_ + third_hysteresis_margin_;
    third_hysteresis_min_y_ = third_min_y_ - third_hysteresis_margin_;
    third_hysteresis_max_y_ = third_max_y_ + third_hysteresis_margin_;

    rm_task_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/rm_task", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rm_task_value_.store(msg->data);
        });
    area_status_pub_ =
        this->create_publisher<std_msgs::msg::Int32>("/area_status", 10);

    costmap_client_ =
        this->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    gimbal_angle_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/gimbal_angle", 10);
    self_save_status_pub_ =
        this->create_publisher<std_msgs::msg::Int32>("/self_save_status", 10);

    cmd_vel_save_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_save", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SpecialArea::checkRobotPosition, this));

    save_timer_ = this->create_wall_timer(std::chrono::milliseconds(250),
                                          std::bind(&SpecialArea::save, this));
    inside_active = false;
    third_inside_active_ = false;
    current_gimbal_angle_ = 0.0;
  }

private:
  double special_last_reset_sec_ = 0.0;
  const double special_reset_interval_sec_ = 10 * 60.0;
  double special_active_duration_sec_ = 6.0;
  double special_trigger_start_sec_ = -1.0;
  double third_p1_x_, third_p1_y_;
  double third_p2_x_, third_p2_y_;
  double third_min_x_, third_max_x_;
  double third_min_y_, third_max_y_;
  double third_hysteresis_min_x_, third_hysteresis_max_x_;
  double third_hysteresis_min_y_, third_hysteresis_max_y_;
  double third_hysteresis_margin_;
  bool third_inside_active_;
  double special_p1_x_, special_p1_y_;
  double special_p2_x_, special_p2_y_;
  double special_p3_x_, special_p3_y_;
  double special_p4_x_, special_p4_y_;
  double special_hysteresis_margin_ = 0.2;
  std::array<std::pair<double, double>, 4> special_polygon_;
  std::array<std::pair<double, double>, 4> special_hysteresis_polygon_;
  bool special_area_has_triggered_ = false;
  bool special_inside_active_ = false;
  double special_gimbal_angle_ = 0.0;
  bool fight_outpost_on_island_ = false;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr special_area_angle_pub_;
  double left_top_x, left_top_y, right_bottom_x, right_bottom_y;
  double min_x, min_y, max_x, max_y;
  double hysteresis_min_x, hysteresis_max_x, hysteresis_min_y, hysteresis_max_y;
  double transform_tolerance_ = 0.1;
  bool inside_active;
  double current_gimbal_angle_;
  std::string global_frame_ = "map";
  std::string robot_base_frame_ = "base_link";
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_save_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr self_save_status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr gimbal_timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::string service_name_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr area_status_pub_;
  double self_save_sample_radius_ = 3.0;
  int self_save_sample_directions_ = 18;
  double self_save_speed_ = 1.2;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rm_task_sub_;
  std::atomic<int> rm_task_value_{0};
  bool isPointInPolygon(
      double x, double y,
      const std::array<std::pair<double, double>, 4> &polygon) const {
    bool inside = false;
    int n = static_cast<int>(polygon.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = polygon[i].first, yi = polygon[i].second;
      double xj = polygon[j].first, yj = polygon[j].second;

      if (((yi > y) != (yj > y)) &&
          (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    return inside;
  }

  std::array<std::pair<double, double>, 4>
  offsetConvexPolygon(const std::array<std::pair<double, double>, 4> &polygon,
                      double margin) const {
    std::array<std::pair<double, double>, 4> offset_poly;
    int n = static_cast<int>(polygon.size());
    std::array<std::tuple<double, double, double>, 4> lines;

    for (int i = 0; i < n; ++i) {
      double x1 = polygon[i].first;
      double y1 = polygon[i].second;
      double x2 = polygon[(i + 1) % n].first;
      double y2 = polygon[(i + 1) % n].second;
      double dx = x2 - x1;
      double dy = y2 - y1;
      double len = std::hypot(dx, dy);
      if (len < 1e-6) {
        lines[i] = {0, 0, 0};
        continue;
      }

      double nx = -dy / len;
      double ny = dx / len;
      double px = x1 + nx * margin;
      double py = y1 + ny * margin;
      double a = dy;
      double b = -dx;
      double c = -dy * px + dx * py;

      lines[i] = {a, b, c};
    }

    for (int i = 0; i < n; ++i) {
      int j = (i - 1 + n) % n;
      auto [a1, b1, c1] = lines[j];
      auto [a2, b2, c2] = lines[i];
      double det = a1 * b2 - a2 * b1;
      if (std::fabs(det) < 1e-6) {
        offset_poly[i] = polygon[i];
        continue;
      }

      double x = (b1 * c2 - b2 * c1) / det;
      double y = (a2 * c1 - a1 * c2) / det;
      offset_poly[i] = {x, y};
    }

    return offset_poly;
  }

  int countLeadingObstacleCount(const std::vector<float> &cost_list) const {
    const float OBSTACLE_THRESHOLD = 240.0f;
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

  double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double normalizeAngleDeg(double angle_deg) {
    while (angle_deg > 180.0) {
      angle_deg -= 360.0;
    }
    while (angle_deg < -180.0) {
      angle_deg += 360.0;
    }
    return angle_deg;
  }

  void publishSelfSaveStatus(int status) {
    std_msgs::msg::Int32 msg;
    msg.data = status;
    self_save_status_pub_->publish(msg);
  }

  void checkRobotPosition() {
    const double sec = this->now().seconds();
    if (sec - special_last_reset_sec_ >= special_reset_interval_sec_) {
      special_area_has_triggered_ = false;
      special_last_reset_sec_ = sec;
      RCLCPP_INFO(this->get_logger(),
                  "Special area: 10min timeout -> RESET status.");
    }
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::TransformStamped fake_tf;

    try {
      transformStamped = tf_buffer_->lookupTransform(
          global_frame_, robot_base_frame_, rclcpp::Time(0));
      fake_tf = tf_buffer_->lookupTransform(robot_base_frame_, "fake_base_link",
                                            rclcpp::Time(0));
      double robot_x = transformStamped.transform.translation.x;
      double robot_y = transformStamped.transform.translation.y;
      double rel_yaw = tf2::getYaw(fake_tf.transform.rotation);
      double gimbal_angle = normalizeAngle(rel_yaw);
      gimbal_angle = gimbal_angle * 180 / M_PI;
      double special_angle = normalizeAngle(rel_yaw - 20.0 * M_PI / 180.0);
      special_angle = special_angle * 180.0 / M_PI;
      current_gimbal_angle_ = gimbal_angle;
      bool now_inside_small = (robot_x > min_x && robot_x < max_x &&
                               robot_y > min_y && robot_y < max_y);
      bool now_inside_large =
          (robot_x > hysteresis_min_x && robot_x < hysteresis_max_x &&
           robot_y > hysteresis_min_y && robot_y < hysteresis_max_y);

      if (now_inside_small) {
        inside_active = true;
      } else if (!now_inside_large) {
        inside_active = false;
      }

      bool now_third_inside_small =
          (robot_x > third_min_x_ && robot_x < third_max_x_ &&
           robot_y > third_min_y_ && robot_y < third_max_y_);
      bool now_third_inside_large = (robot_x > third_hysteresis_min_x_ &&
                                     robot_x < third_hysteresis_max_x_ &&
                                     robot_y > third_hysteresis_min_y_ &&
                                     robot_y < third_hysteresis_max_y_);

      if (now_third_inside_small) {
        third_inside_active_ = true;
      } else if (!now_third_inside_large) {
        third_inside_active_ = false;
      }

      bool special_inside_small =
          isPointInPolygon(robot_x, robot_y, special_polygon_);
      bool special_inside_large =
          isPointInPolygon(robot_x, robot_y, special_hysteresis_polygon_);
      const double now_sec = this->now().seconds();

      if (special_inside_small && !special_area_has_triggered_) {
        special_inside_active_ = true;
        special_area_has_triggered_ = true;
        special_trigger_start_sec_ = now_sec;

        RCLCPP_INFO(this->get_logger(),
                    "Special area triggered, start %.1f sec timer.",
                    special_active_duration_sec_);
      }

      if (special_inside_active_) {
        const bool special_exited = !special_inside_large;
        const bool special_timeout = (now_sec - special_trigger_start_sec_) >=
                                     special_active_duration_sec_;

        if (special_exited || special_timeout) {
          special_inside_active_ = false;
        }
      }

      int area_status = 0;
      const int rm_task = rm_task_value_.load();
      const bool force_area_status_1 = (rm_task == 1);
      const bool force_area_status_3 = (rm_task == 2);
      const bool force_area_status_2 = (rm_task == 4);
      const bool special_region_requests_area_status_2 =
          !fight_outpost_on_island_ && special_inside_active_;
      if (force_area_status_3) {
        area_status = 3;
      } else if (force_area_status_1) {
        area_status = 1;
      } else if (special_region_requests_area_status_2 || force_area_status_2) {
        area_status = 2;
      } else if (inside_active || third_inside_active_) {
        area_status = 1;
      } else {
        area_status = 0;
      }

      std_msgs::msg::Int32 area_msg;
      area_msg.data = area_status;
      area_status_pub_->publish(area_msg);

      std_msgs::msg::Float32 angle_msg;
      bool should_publish_angle = false;

      if (area_status == 3) {
        if (robot_x < 10.0) {
          angle_msg.data = static_cast<float>(
              normalizeAngleDeg(current_gimbal_angle_ + 90.0));
        } else {
          angle_msg.data = static_cast<float>(
              normalizeAngleDeg(current_gimbal_angle_ - 90.0));
        }

        should_publish_angle = true;
      } else if (area_status == 2) {
        if (force_area_status_2) {
          angle_msg.data = static_cast<float>(
              normalizeAngleDeg(current_gimbal_angle_ + 10.0));
        } else {
          angle_msg.data = static_cast<float>(special_angle);
        }
        should_publish_angle = true;
      } else if (area_status == 1) {
        angle_msg.data = static_cast<float>(current_gimbal_angle_);
        should_publish_angle = true;
      }

      if (should_publish_angle) {
        gimbal_angle_pub_->publish(angle_msg);
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  void parseCostmapAndQuery(const nav2_msgs::msg::Costmap &costmap) {
    geometry_msgs::msg::TransformStamped tf_stamped;

    tf_stamped = tf_buffer_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance_));
    double x = tf_stamped.transform.translation.x;
    double y = tf_stamped.transform.translation.y;
    tf2::Quaternion q(
        tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y,
        tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w);
    double pitch, roll, robot_yaw_map;
    tf2::Matrix3x3(q).getEulerYPR(robot_yaw_map, pitch, roll);
    int c = parseCostmap(costmap, x, y);
    if (c <= 220) {
      publishSelfSaveStatus(0);
      publishZeroVelocity();
      return;
    }
    publishSelfSaveStatus(1);
    float min_avg_cost = 256.0f;
    float best_direction_rad = 0.0f;
    bool has_safe_direction = false;
    int min_leading_obstacle_count = std::numeric_limits<int>::max();

    for (int i = 0; i < self_save_sample_directions_; ++i) {
      float direction_rad =
          2.0f * static_cast<float>(M_PI) * i / self_save_sample_directions_;
      std::vector<float> cost_list =
          getCostListInDirection(costmap, x, y, direction_rad);

      if (cost_list.empty()) {
        continue;
      }

      int leading_obstacle_count = countLeadingObstacleCount(cost_list);
      float avg_cost = calculateAvgCost(cost_list);

      if (!has_safe_direction ||
          leading_obstacle_count < min_leading_obstacle_count ||
          (leading_obstacle_count == min_leading_obstacle_count &&
           avg_cost < min_avg_cost)) {
        min_leading_obstacle_count = leading_obstacle_count;
        min_avg_cost = avg_cost;
        best_direction_rad = direction_rad;
        has_safe_direction = true;
      }
    }

    geometry_msgs::msg::Twist cmd_vel;
    if (has_safe_direction) {
      float dx_map = self_save_speed_ * std::cos(best_direction_rad);
      float dy_map = self_save_speed_ * std::sin(best_direction_rad);

      cmd_vel =
          convertMapDirectionToBaseLinkSpeed(dx_map, dy_map, robot_yaw_map);
    } else {
      cmd_vel.linear.x = 0.0f;
      cmd_vel.linear.y = 0.0f;
      cmd_vel.angular.z = 0.0f;
    }
    cmd_vel_save_pub_->publish(cmd_vel);
  }

  void save() {
    if (!costmap_client_->wait_for_service(std::chrono::milliseconds(200))) {
      return;
    }
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    using ServiceResponseFuture =
        rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture;
    auto response_callback = [this](ServiceResponseFuture future) {
      try {
        const auto &costmap = future.get()->map;
        parseCostmapAndQuery(costmap);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败: %s", e.what());
      }
    };
    costmap_client_->async_send_request(request, response_callback);
  }

  geometry_msgs::msg::Twist
  convertMapDirectionToBaseLinkSpeed(float dx_map, float dy_map,
                                     float yaw_map) const {
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x =
        dx_map * std::cos(-yaw_map) - dy_map * std::sin(-yaw_map);
    cmd_vel.linear.y =
        dx_map * std::sin(-yaw_map) + dy_map * std::cos(-yaw_map);
    cmd_vel.linear.z = 0.0f;
    cmd_vel.angular.z = 0.0f;
    float speed_mag = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
    if (speed_mag > 1e-6f) {
      cmd_vel.linear.x /= speed_mag;
      cmd_vel.linear.y /= speed_mag;
      cmd_vel.linear.x *= static_cast<float>(self_save_speed_);
      cmd_vel.linear.y *= static_cast<float>(self_save_speed_);
    }
    return cmd_vel;
  }

  void publishZeroVelocity() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;
    cmd_vel_save_pub_->publish(twist);
  }

  std::vector<float>
  getCostListInDirection(const nav2_msgs::msg::Costmap &costmap,
                         float robot_x_map, float robot_y_map,
                         float direction_rad) const {
    std::vector<float> cost_list;
    const auto &meta = costmap.metadata;
    float resolution = meta.resolution;
    float origin_x = meta.origin.position.x;
    float origin_y = meta.origin.position.y;
    int width = meta.size_x;
    int height = meta.size_y;
    float sample_step = resolution;
    int total_sample_points =
        static_cast<int>(self_save_sample_radius_ / sample_step);
    if (total_sample_points < 1)
      total_sample_points = 1;

    for (int step = 1; step <= total_sample_points; ++step) {
      float sample_x =
          robot_x_map + step * sample_step * std::cos(direction_rad);
      float sample_y =
          robot_y_map + step * sample_step * std::sin(direction_rad);
      int grid_x = static_cast<int>((sample_x - origin_x) / resolution);
      int grid_y = static_cast<int>((sample_y - origin_y) / resolution);
      size_t idx = static_cast<size_t>(grid_y) * static_cast<size_t>(width) +
                   static_cast<size_t>(grid_x);
      if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height ||
          idx >= costmap.data.size()) {
        cost_list.push_back(255.0f);
      } else {
        cost_list.push_back(static_cast<float>(costmap.data[idx]));
      }
    }
    return cost_list;
  }

  bool isDirectionSafe(const std::vector<float> &cost_list) const {
    const int REQUIRED_DECREASE_POINTS = 3;
    const float COST_THRESHOLD = 150.0f;
    if (cost_list.size() <= static_cast<size_t>(REQUIRED_DECREASE_POINTS))
      return false;
    for (size_t i = 0; i + REQUIRED_DECREASE_POINTS < cost_list.size(); ++i) {
      if (cost_list[i] >= COST_THRESHOLD)
        continue;
      bool is_decreasing = true;
      for (int j = 0; j < REQUIRED_DECREASE_POINTS; ++j) {
        if (cost_list[i + j + 1] > cost_list[i + j]) {
          is_decreasing = false;
          break;
        }
      }
      if (is_decreasing)
        return true;
    }
    return false;
  }

  float calculateAvgCost(const std::vector<float> &cost_list) const {
    if (cost_list.empty())
      return 255.0f;
    float sum = 0.0f;
    for (float c : cost_list)
      sum += c;
    return sum / cost_list.size();
  }

  void publishGimbalAngle() {
    if (inside_active || third_inside_active_) {
      std_msgs::msg::Float32 angle_msg;
      angle_msg.data = current_gimbal_angle_;
      gimbal_angle_pub_->publish(angle_msg);
    }
  }

  int parseCostmap(const nav2_msgs::msg::Costmap &costmap, double x, double y) {
    const auto &metadata = costmap.metadata;
    float resolution = metadata.resolution;
    float origin_x = metadata.origin.position.x;
    float origin_y = metadata.origin.position.y;
    int width = metadata.size_x;
    int height = metadata.size_y;
    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
      return 256;
    }

    size_t index = static_cast<size_t>(grid_y) * static_cast<size_t>(width) +
                   static_cast<size_t>(grid_x);
    if (index >= costmap.data.size()) {
      return 256;
    }
    unsigned char cost = costmap.data[index];
    int c = cost;
    return c;
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<SpecialArea>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SpecialArea)
