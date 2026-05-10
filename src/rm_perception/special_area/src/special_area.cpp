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

class SpecialArea : public rclcpp::Node
{
public:
    SpecialArea(const rclcpp::NodeOptions &node_options) : Node("special_area", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "SpecialArea node has been started.");

        // 原始区域参数
        this->declare_parameter("left_top_x", 8.641380);
        this->declare_parameter("left_top_y", 2.241885);
        this->declare_parameter("right_bottom_x", 5.596358);
        this->declare_parameter("right_bottom_y", -1.603606);
        this->declare_parameter("service_name", "/global_costmap/get_costmap");
        this->get_parameter("left_top_x", left_top_x);
        this->get_parameter("left_top_y", left_top_y);
        this->get_parameter("right_bottom_x", right_bottom_x);
        this->get_parameter("right_bottom_y", right_bottom_y);
        this->get_parameter("service_name", service_name_);
        // 原始小区域
        min_x = std::min(left_top_x, right_bottom_x);
        max_x = std::max(left_top_x, right_bottom_x);
        min_y = std::min(left_top_y, right_bottom_y);
        max_y = std::max(left_top_y, right_bottom_y);

        // 滞回大区域（向外拓展0.2）
        hysteresis_min_x = min_x - 0.2;
        hysteresis_max_x = max_x + 0.2;
        hysteresis_min_y = min_y - 0.2;
        hysteresis_max_y = max_y + 0.2;

        costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        spin_pub_ = this->create_publisher<std_msgs::msg::Int32>("/spin_status", 10);
        gimbal_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gimbal_angle", 10);
        self_save_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/self_save_status", 10);

        cmd_vel_save_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_save", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpecialArea::checkRobotPosition, this)
        );

        gimbal_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpecialArea::publishGimbalAngle, this)
        );
        save_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(400),  // 200ms 跑一次，够快够稳
            std::bind(&SpecialArea::save, this)
        );
        inside_active = false;
        current_gimbal_angle_ = 0.0;
    }

private:

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
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr spin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr self_save_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr gimbal_timer_;
    rclcpp::TimerBase::SharedPtr save_timer_;  
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    std::string service_name_;

    double self_save_sample_radius_ = 3.0;
    int self_save_sample_directions_ = 18;
    double self_save_speed_ = 1.5;
    double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }
    void publishSelfSaveStatus(int status)
    {
        std_msgs::msg::Int32 msg;
        msg.data = status;
        self_save_status_pub_->publish(msg);
    }
    void checkRobotPosition()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        geometry_msgs::msg::TransformStamped fake_tf;

        try {
            transformStamped = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, rclcpp::Time(0));
            fake_tf = tf_buffer_->lookupTransform(robot_base_frame_, "fake_base_link", rclcpp::Time(0));

            double robot_x = transformStamped.transform.translation.x;
            double robot_y = transformStamped.transform.translation.y;
            double rel_yaw = tf2::getYaw(fake_tf.transform.rotation);
            double gimbal_angle = normalizeAngle(rel_yaw + M_PI_2);
            gimbal_angle = gimbal_angle * 180 / M_PI;

            // 保存最新角度
            current_gimbal_angle_ = gimbal_angle;

            // 滞回判断
            bool now_inside_small = (robot_x > min_x && robot_x < max_x &&
                                    robot_y > min_y && robot_y < max_y);
            bool now_inside_large = (robot_x > hysteresis_min_x && robot_x < hysteresis_max_x &&
                                    robot_y > hysteresis_min_y && robot_y < hysteresis_max_y);

            if (now_inside_small) {
                inside_active = true;
            } else if (!now_inside_large) {
                inside_active = false;
            }

            std_msgs::msg::Int32 msg;
            msg.data = inside_active ? 0 : 1;
            spin_pub_->publish(msg);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
    void save(){
        //   while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
        //     if (!rclcpp::ok()) {
        //       RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        //       return;
        //     }
        //     RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
        //   }
          if (!costmap_client_->wait_for_service(std::chrono::milliseconds(200))) {
            // publishSelfSaveStatus(0);
            return;
         }
          auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
          auto result = costmap_client_->async_send_request(request);
          if (result.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready) {
            // publishSelfSaveStatus(0);
            return;
          }
          auto costmap = result.get()->map;
        
          geometry_msgs::msg::TransformStamped tf_stamped;

        //   try {
          tf_stamped = tf_buffer_->lookupTransform(
                global_frame_,          // target frame
                robot_base_frame_,      // source frame
                tf2::TimePointZero,
                tf2::durationFromSec(transform_tolerance_));
          double x = tf_stamped.transform.translation.x;
          double y = tf_stamped.transform.translation.y;
          tf2::Quaternion q(
            tf_stamped.transform.rotation.x,
            tf_stamped.transform.rotation.y,
            tf_stamped.transform.rotation.z,
            tf_stamped.transform.rotation.w);
        double pitch, roll, robot_yaw_map;
        tf2::Matrix3x3(q).getEulerYPR(robot_yaw_map, pitch, roll);

          int c = parseCostmap(costmap, x, y);
          if (c <= 220) {
            publishSelfSaveStatus(0);
            publishZeroVelocity();
            return;
          } 
          publishSelfSaveStatus(1);
          float min_avg_cost = 255.0f;
          float best_direction_rad = 0.0f;
          bool has_safe_direction = false;
        for (int i = 0; i < self_save_sample_directions_; ++i) {
            float direction_rad = 2.0f * static_cast<float>(M_PI) * i / self_save_sample_directions_;
            std::vector<float> cost_list = getCostListInDirection(costmap, x, y, direction_rad);
            if (isDirectionSafe(cost_list)) {
                float avg_cost = calculateAvgCost(cost_list);
                if (avg_cost < min_avg_cost) {
                    min_avg_cost = avg_cost;
                    best_direction_rad = direction_rad;
                    has_safe_direction = true;
                }
            }
        }
        geometry_msgs::msg::Twist cmd_vel;
        if (has_safe_direction) {

            float dx_map = self_save_speed_ * std::cos(best_direction_rad);
            float dy_map = self_save_speed_ * std::sin(best_direction_rad);
        
            cmd_vel = convertMapDirectionToBaseLinkSpeed(
                dx_map, dy_map, robot_yaw_map
            );
        
        } else {
            cmd_vel.linear.x = 0.0f;
            cmd_vel.linear.y = 0.0f;
            cmd_vel.angular.z = 0.0f;
        }
        cmd_vel_save_pub_->publish(cmd_vel);
    }
    geometry_msgs::msg::Twist convertMapDirectionToBaseLinkSpeed(
        float dx_map, float dy_map, float yaw_map) const
    {
        geometry_msgs::msg::Twist cmd_vel;
    
        cmd_vel.linear.x = dx_map * std::cos(-yaw_map) - dy_map * std::sin(-yaw_map);
        cmd_vel.linear.y = dx_map * std::sin(-yaw_map) + dy_map * std::cos(-yaw_map);
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
    void publishZeroVelocity()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        cmd_vel_save_pub_->publish(twist);
    }
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
    void publishGimbalAngle()
    {
        if (inside_active) {
            std_msgs::msg::Float32 angle_msg;
            angle_msg.data = current_gimbal_angle_;
            gimbal_angle_pub_->publish(angle_msg);
        }
    }
    int parseCostmap(const nav2_msgs::msg::Costmap& costmap,double x, double y) {
        const auto& metadata = costmap.metadata;
        float resolution = metadata.resolution;         
        float origin_x = metadata.origin.position.x;    
        float origin_y = metadata.origin.position.y;    
        int width = metadata.size_x;                     
        int height = metadata.size_y;                     
      
        int grid_x = static_cast<int>((x - origin_x) / resolution);
        int grid_y = static_cast<int>((y - origin_y) / resolution);
      
        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
          RCLCPP_WARN(this->get_logger(), 
            "========== 查询结果 ==========\n"
            "目标点 (%.2f, %.2f) 超出代价地图范围！\n"
            "栅格坐标: (%d, %d)，地图尺寸: %dx%d栅格\n"
            "地图范围: X[%.2f, %.2f], Y[%.2f, %.2f]\n"
            "==================================",
            x, y, grid_x, grid_y, width, height,
            origin_x, origin_x + width*resolution,
            origin_y, origin_y + height*resolution);
          return 256;
        }
      
        size_t index = static_cast<size_t>(grid_y) * static_cast<size_t>(width) + static_cast<size_t>(grid_x);
        if (index >= costmap.data.size()) {
          RCLCPP_ERROR(this->get_logger(), "无效栅格索引: %zu（总长度：%zu）", 
            index, costmap.data.size());
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