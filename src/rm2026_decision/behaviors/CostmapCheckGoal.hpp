#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include <future>

namespace rm2026_decision {

/**
 * @brief 代价地图检查行为节点（同步、无 onStart）
 * 在 game_progress==4 时同步请求代价地图，检查导航目标点代价值；
 * 若存在障碍则搜索无障碍圆弧段并更新黑板 @nav_goal / @nav_goal_requested。
 * 若当前点代价值>250或越界则在本 tick 内阻塞执行自救直至安全或超时。
 * 逻辑与标注完全套用 decision_node 代价地图检查与自救实现。
 */
class CostmapCheckGoal : public BT::SyncActionNode
{
public:
    // 与 DecisionNode 中 CostmapConfig 常量一致
    static constexpr float ANGLE_STEP_MIN = M_PI / 180.0f;   // 1度
    static constexpr float ANGLE_STEP_MAX = M_PI / 36.0f;     // 5度
    static constexpr float ANGLE_TOLERANCE = std::numeric_limits<float>::epsilon() * 100.0f;
    // 自救常量
    static constexpr float SELF_SAVE_SAMPLE_RADIUS = 3.0f;
    static constexpr int SELF_SAVE_SAMPLE_DIRECTIONS = 18;
    static constexpr float SELF_SAVE_SPEED_MAGNITUDE = 1.0f;
    static constexpr int SELF_SAVE_STOP_THRESHOLD = 150;
    static constexpr int SELF_SAVE_MAX_DURATION_MS = 5000;
    static constexpr int SELF_SAVE_TRIGGER_THRESHOLD = 250;

    CostmapCheckGoal(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
        , node_(nullptr)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rclcpp::Node::SharedPtr>("ros_node", "ROS2 node instance"),
            BT::InputPort<int>("game_progress", "@game_progress", "Game progress, only 4 triggers check"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_goal", "@nav_goal", "Navigation goal(project frame)"),
            BT::InputPort<float>("current_pose_x", "@current_pose_x", "Current robot X coordinate(meters)"),
            BT::InputPort<float>("current_pose_y", "@current_pose_y", "Current robot Y coordinate(meters)"),
            BT::InputPort<nav2_msgs::msg::Costmap>("costmap", "@costmap", "Costmap from blackboard, 1Hz by decision_node"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goalOut", "Navigation goal(updated, write to blackboard)"),
            BT::OutputPort<bool>("nav_goal_requested", "Navigation goal requested"),
            BT::OutputPort<int>("self_save_statusOut",  "Self-save status: 0=not in self-save, 1=in self-save"),
            BT::OutputPort<geometry_msgs::msg::Twist>("cmd_vel_saveOut", "Self-save cmd_vel, decision_node publishes when status==1"),
        };
    }

    BT::NodeStatus tick() override
    {
        auto node_input = getInput<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_input || !node_input.value()) {
            RCLCPP_ERROR(rclcpp::get_logger("CostmapCheckGoal"), "Failed to get ROS2 node instance");
            return BT::NodeStatus::FAILURE;
        }
        node_ = node_input.value();

        auto gp = getInput<int>("game_progress");
        if (!gp || gp.value() != 4) {
            return BT::NodeStatus::SUCCESS;
        }

        auto nav_goal_in = getInput<geometry_msgs::msg::PoseStamped>("nav_goal");
        if (!nav_goal_in) {
            RCLCPP_DEBUG(node_->get_logger(), "CostmapCheckGoal: no @nav_goal, skip");
            return BT::NodeStatus::SUCCESS;
        }

        loadConfigFromNode();
        target_x_ = nav_goal_in.value().pose.position.x + offset_x_;
        target_y_ = nav_goal_in.value().pose.position.y + offset_y_;
        map_frame_ = node_->has_parameter("map_frame") ? node_->get_parameter("map_frame").as_string() : "map";

        auto costmap_in = getInput<nav2_msgs::msg::Costmap>("costmap");
        if (!costmap_in) {
            RCLCPP_DEBUG(node_->get_logger(), "CostmapCheckGoal: no @costmap on blackboard, skip");
            return BT::NodeStatus::SUCCESS;
        }
        nav2_msgs::msg::Costmap costmap = costmap_in.value();

        need_self_save_ = false;
        loadSelfSaveConfig();  // 提前加载自救参数，供防抖判断使用
        parseCostmapAndCheckGoal(costmap);
        //RCLCPP_INFO(node_->get_logger(), "Self重新再次tick1");
        setOutput("self_save_statusOut", 0); 
        if (need_self_save_) {
            loadSelfSaveConfig();
            ensureTfBuffer();
            setOutput("self_save_statusOut", 1);   // 进入自救：黑板设为 1
            bool ok = runBlockingSelfSave(costmap);
            setOutput("cmd_vel_saveOut", zeroTwist());  // 结束自救：黑板速度置零，由 decision_node 发布
            setOutput("self_save_statusOut", 0);   // 结束自救或特殊情况结束：黑板设为 0
            if (!ok) {
                RCLCPP_ERROR(node_->get_logger(), "Self-save timeout(%dms), exit", SELF_SAVE_MAX_DURATION_MS);
            } else {
                RCLCPP_INFO(node_->get_logger(), "Self-save done");
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    void loadConfigFromNode()
    {
        offset_x_ = node_->get_parameter("project_to_nav_offset_x").as_double();
        offset_y_ = node_->get_parameter("project_to_nav_offset_y").as_double();
        search_radius_step_ = node_->get_parameter("search_radius_step").as_double();
        max_search_radius_ = node_->get_parameter("max_search_radius").as_double();
        sector_angle_ = node_->get_parameter("sector_angle").as_double();
        obstacle_threshold_ = node_->get_parameter("obstacle_threshold").as_int();
    }

    void loadSelfSaveConfig()
    {
        if (self_save_config_loaded_) return;
        self_save_config_loaded_ = true;
        if (node_->has_parameter("self_save_stop_threshold")) {
            self_save_stop_threshold_ = node_->get_parameter("self_save_stop_threshold").as_int();
        }
        if (node_->has_parameter("self_save_sample_directions")) {
            self_save_sample_directions_ = node_->get_parameter("self_save_sample_directions").as_int();
        }
        if (node_->has_parameter("self_save_speed_magnitude")) {
            self_save_speed_magnitude_ = node_->get_parameter("self_save_speed_magnitude").as_double();
        }
        if (node_->has_parameter("self_save_sample_radius")) {
            self_save_sample_radius_ = node_->get_parameter("self_save_sample_radius").as_double();
        }
        if (node_->has_parameter("self_save_trigger_threshold")) {
            self_save_trigger_threshold_ = node_->get_parameter("self_save_trigger_threshold").as_int();
        }
        if (node_->has_parameter("self_save_debounce_ticks")) {
            self_save_debounce_ticks_ = node_->get_parameter("self_save_debounce_ticks").as_int();
            if (self_save_debounce_ticks_ < 1) self_save_debounce_ticks_ = 1;
        }
        if (self_save_stop_threshold_ >= self_save_trigger_threshold_) {
            self_save_stop_threshold_ = self_save_trigger_threshold_ - 1;
            RCLCPP_WARN(node_->get_logger(), "Self-save stop_threshold adjusted to %d", self_save_stop_threshold_);
        }
    }

    void ensureTfBuffer()
    {
        if (!tf_buffer_) {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);
        }
    }

    static geometry_msgs::msg::Twist zeroTwist()
    {
        geometry_msgs::msg::Twist z;
        z.linear.x = z.linear.y = z.linear.z = 0.0;
        z.angular.x = z.angular.y = z.angular.z = 0.0;
        return z;
    }

    // 阻塞式自救循环，返回 true 表示正常结束，false 表示超时
    bool runBlockingSelfSave(const nav2_msgs::msg::Costmap& costmap)
    {
        rclcpp::Rate loop_rate(3.0);
        auto start_time = node_->get_clock()->now();
        RCLCPP_WARN(node_->get_logger(), "Self-save started...");

        while (rclcpp::ok()) {
            rclcpp::Duration elapsed = node_->get_clock()->now() - start_time;
            if (elapsed > rclcpp::Duration::from_seconds(SELF_SAVE_MAX_DURATION_MS / 1000.0)) {
                return false;
            }

            double robot_x_map = 0.0, robot_y_map = 0.0, robot_yaw_map = 0.0;
            try {
                auto transform = tf_buffer_->lookupTransform(
                    "map", "base_link", tf2::TimePointZero, std::chrono::milliseconds(50));
                robot_x_map = transform.transform.translation.x;
                robot_y_map = transform.transform.translation.y;
                tf2::Quaternion q(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
                double pitch, roll;
                tf2::Matrix3x3(q).getEulerYPR(robot_yaw_map, pitch, roll);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR(node_->get_logger(), "TF map->base_link failed: %s", ex.what());
                setOutput("cmd_vel_saveOut", zeroTwist());
                loop_rate.sleep();
                continue;
            }

            const auto& meta = costmap.metadata;
            float resolution = meta.resolution;
            float origin_x = meta.origin.position.x;
            float origin_y = meta.origin.position.y;
            int width = meta.size_x;
            int height = meta.size_y;
            int grid_x_self = static_cast<int>((robot_x_map - origin_x) / resolution);
            int grid_y_self = static_cast<int>((robot_y_map - origin_y) / resolution);
            if (grid_x_self < 0 || grid_x_self >= width || grid_y_self < 0 || grid_y_self >= height) {
                setOutput("cmd_vel_saveOut", zeroTwist());
                loop_rate.sleep();
                continue;
            }
            size_t index_self = static_cast<size_t>(grid_y_self) * static_cast<size_t>(width) + static_cast<size_t>(grid_x_self);
            if (index_self >= costmap.data.size()) {
                setOutput("cmd_vel_saveOut", zeroTwist());
                loop_rate.sleep();
                continue;
            }
            unsigned char cost_self = costmap.data[index_self];

            if (cost_self < static_cast<unsigned char>(self_save_stop_threshold_)) {
                RCLCPP_INFO(node_->get_logger(), "Self-save done, cost=%d < %d",
                            static_cast<int>(cost_self), self_save_stop_threshold_);
                return true;
            }

            float min_avg_cost = 255.0f;
            float best_direction_rad = 0.0f;
            bool has_safe_direction = false;
            for (int i = 0; i < self_save_sample_directions_; ++i) {
                float direction_rad = 2.0f * static_cast<float>(M_PI) * i / self_save_sample_directions_;
                std::vector<float> cost_list = getCostListInDirection(costmap, robot_x_map, robot_y_map, direction_rad);
                if (isDirectionSafe(cost_list)) {
                    float avg_cost = calculateAvgCost(cost_list);
                    if (avg_cost < min_avg_cost) {
                        min_avg_cost = avg_cost;
                        best_direction_rad = direction_rad;
                        has_safe_direction = true;
                    }
                }
            }
            if (has_safe_direction) {
                float dx_map = self_save_speed_magnitude_ * std::cos(best_direction_rad);
                float dy_map = self_save_speed_magnitude_ * std::sin(best_direction_rad);
                geometry_msgs::msg::Twist cmd_vel = convertMapDirectionToBaseLinkSpeed(dx_map, dy_map, robot_yaw_map);
                setOutput("cmd_vel_saveOut", cmd_vel);
            } else {
                setOutput("cmd_vel_saveOut", zeroTwist());  // 无安全方向时发零速
            }
            loop_rate.sleep();
        }
        return false;
    }

    bool isPointSafe(const nav2_msgs::msg::Costmap& costmap, double x, double y) const
    {
        const auto& meta = costmap.metadata;
        int gx = static_cast<int>((x - meta.origin.position.x) / meta.resolution);
        int gy = static_cast<int>((y - meta.origin.position.y) / meta.resolution);
        if (gx < 0 || static_cast<unsigned int>(gx) >= meta.size_x ||
            gy < 0 || static_cast<unsigned int>(gy) >= meta.size_y) {
            return false;
        }
        size_t idx = static_cast<size_t>(gy) * meta.size_x + gx;
        return costmap.data[idx] < static_cast<unsigned char>(obstacle_threshold_);
    }

    bool findSafeArcCenter(const nav2_msgs::msg::Costmap& costmap,
                          double center_x, double center_y, double radius,
                          double sector_angle,
                          double& out_arc_center_x, double& out_arc_center_y) const
    {
        const auto& meta = costmap.metadata;
        double res = meta.resolution;
        float angle_step = static_cast<float>(res / radius);
        angle_step = std::max(angle_step, ANGLE_STEP_MIN);
        angle_step = std::min(angle_step, ANGLE_STEP_MAX);
        const float start_angle_step = std::max(ANGLE_STEP_MAX, static_cast<float>(M_PI / 18.0));

        for (float start_angle = 0.0f; start_angle < 2.0f * static_cast<float>(M_PI); start_angle += start_angle_step) {
            int safe_point_count = 0;
            float first_safe_angle = 0.0f;
            float last_safe_angle = 0.0f;
            float end_angle = start_angle + static_cast<float>(sector_angle) + ANGLE_TOLERANCE;

            for (float angle = start_angle; angle <= end_angle; angle += angle_step) {
                float x = center_x + radius * std::cos(angle);
                float y = center_y + radius * std::sin(angle);
                if (isPointSafe(costmap, x, y)) {
                    if (safe_point_count == 0) first_safe_angle = angle;
                    safe_point_count++;
                    last_safe_angle = angle;
                } else {
                    safe_point_count = 0;
                    continue;
                }
                float covered_angle = last_safe_angle - first_safe_angle;
                if (covered_angle >= static_cast<float>(sector_angle)) {
                    float mid_angle = (first_safe_angle + last_safe_angle) / 2.0f;
                    out_arc_center_x = center_x + radius * std::cos(mid_angle);
                    out_arc_center_y = center_y + radius * std::sin(mid_angle);
                    if (isPointSafe(costmap, out_arc_center_x, out_arc_center_y)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void findAndUpdateSafeGoal(const nav2_msgs::msg::Costmap& costmap)
    {
        double origin_x = target_x_;
        double origin_y = target_y_;
        double required_sector_angle = sector_angle_;

        for (double radius = search_radius_step_; radius <= max_search_radius_; radius += search_radius_step_) {
            double arc_center_x = 0.0;
            double arc_center_y = 0.0;
            if (findSafeArcCenter(costmap, origin_x, origin_y, radius, required_sector_angle, arc_center_x, arc_center_y)) {
                geometry_msgs::msg::PoseStamped new_goal;
                new_goal.header.frame_id = map_frame_;
                new_goal.header.stamp = node_->get_clock()->now();
                new_goal.pose.position.x = arc_center_x - offset_x_;
                new_goal.pose.position.y = arc_center_y - offset_y_;
                new_goal.pose.position.z = 0.0;
                // 缓存当前判断的目标点与选择的新点，下次同目标且仍需重选时直接复用
                last_judged_target_x_ = target_x_;
                last_judged_target_y_ = target_y_;
                last_selected_goal_ = new_goal;
                has_cached_goal_ = true;
                setOutput("nav_goalOut", new_goal);
                setOutput("nav_goal_requested", true);
                RCLCPP_INFO(node_->get_logger(),
                    "找到最优无障碍点--> 原始(%.2f, %.2f) 半径=%.2fm 角度=%.1f° 新目标(%.2f, %.2f)",
                    origin_x - offset_x_, origin_y - offset_y_, radius, required_sector_angle * 180.0 / M_PI,
                    arc_center_x - offset_x_, arc_center_y - offset_y_);
                return;
            }
        }
        RCLCPP_WARN(node_->get_logger(),
            "在%.2fm范围内未找到长度为%.1f°的连续无障碍圆弧段，保持原始目标点",
            max_search_radius_, required_sector_angle * 180.0 / M_PI);
    }

    void parseCostmapAndCheckGoal(const nav2_msgs::msg::Costmap& costmap)
    {
        const auto& metadata = costmap.metadata;
        float resolution = metadata.resolution;
        float origin_x = metadata.origin.position.x;
        float origin_y = metadata.origin.position.y;
        int width = metadata.size_x;
        int height = metadata.size_y;

        int grid_x = static_cast<int>((target_x_ - origin_x) / resolution);
        int grid_y = static_cast<int>((target_y_ - origin_y) / resolution);

        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
            RCLCPP_WARN(node_->get_logger(), "目标点 (%.2f, %.2f) 超出代价地图范围", target_x_, target_y_);
            return;
        }
        size_t index = static_cast<size_t>(grid_y) * static_cast<size_t>(width) + static_cast<size_t>(grid_x);
        if (index >= costmap.data.size()) return;
        unsigned char cost = costmap.data[index];
        if (cost > static_cast<unsigned char>(obstacle_threshold_)) {
            constexpr double kTargetEpsilon = 1e-6;
            bool same_target = has_cached_goal_
                && std::abs(target_x_ - last_judged_target_x_) < kTargetEpsilon
                && std::abs(target_y_ - last_judged_target_y_) < kTargetEpsilon;
            if (same_target) {
                setOutput("nav_goalOut", last_selected_goal_);
                setOutput("nav_goal_requested", true);
                RCLCPP_DEBUG(node_->get_logger(), "目标点(%.2f,%.2f)仍超阈值，复用上次重选点", target_x_, target_y_);
            } else {
                findAndUpdateSafeGoal(costmap);
            }
        } else {
            has_cached_goal_ = false;
        }
        RCLCPP_DEBUG(node_->get_logger(),
            "目标点代价值检查: (%.2f, %.2f) cost=%d %s",
            target_x_, target_y_, static_cast<int>(cost),
            cost > static_cast<unsigned char>(obstacle_threshold_) ? "有障碍物" : "安全");

        // 当前点检查：从端口 @current_pose_x / @current_pose_y 读取（与 decision_node 一致）
        auto current_x_in = getInput<float>("current_pose_x");
        auto current_y_in = getInput<float>("current_pose_y");
        if (!current_x_in || !current_y_in) {
            RCLCPP_DEBUG(node_->get_logger(), "No current_pose_x/y, skip self-save check");
            return;
        }
        float current_x = current_x_in.value()+offset_x_;
        float current_y = current_y_in.value()+offset_y_;
        int grid_x_self = static_cast<int>((current_x - origin_x) / resolution);
        int grid_y_self = static_cast<int>((current_y - origin_y) / resolution);
        if (grid_x_self < 0 || grid_x_self >= width || grid_y_self < 0 || grid_y_self >= height) {
            consecutive_unsafe_ticks_++;
            if (consecutive_unsafe_ticks_ >= self_save_debounce_ticks_) {
                RCLCPP_WARN(node_->get_logger(), "当前点 (%.2f, %.2f) 超出代价地图范围，执行自救", current_x, current_y);
                need_self_save_ = true;
            }
            return;
        }
        size_t index_self = static_cast<size_t>(grid_y_self) * static_cast<size_t>(width) + static_cast<size_t>(grid_x_self);
        if (index_self >= costmap.data.size()) {
            consecutive_unsafe_ticks_++;
            if (consecutive_unsafe_ticks_ >= self_save_debounce_ticks_) {
                RCLCPP_WARN(node_->get_logger(), "当前点栅格索引越界，执行自救");
                need_self_save_ = true;
            }
            return;
        }
        unsigned char cost_self = costmap.data[index_self];
        if (cost_self > static_cast<unsigned char>(self_save_trigger_threshold_)) {
            consecutive_unsafe_ticks_++;
            if (consecutive_unsafe_ticks_ >= self_save_debounce_ticks_) {
                RCLCPP_WARN(node_->get_logger(), "当前点代价值=%d，执行自救", static_cast<int>(cost_self));
                need_self_save_ = true;
            }
        } else {
            consecutive_unsafe_ticks_ = 0;
        }
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

    geometry_msgs::msg::Twist convertMapDirectionToBaseLinkSpeed(
        float dx_map, float dy_map, float yaw_map) const
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = dx_map * std::cos(-yaw_map) - dy_map * std::sin(-yaw_map);
        cmd_vel.linear.y = dx_map * std::sin(-yaw_map) + dy_map * std::cos(-yaw_map);
        cmd_vel.linear.z = 0.0f;
        cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0f;
        float speed_mag = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
        if (speed_mag > 1e-6f) {
            cmd_vel.linear.x /= speed_mag;
            cmd_vel.linear.y /= speed_mag;
            cmd_vel.linear.x *= static_cast<float>(self_save_speed_magnitude_);
            cmd_vel.linear.y *= static_cast<float>(self_save_speed_magnitude_);
        }
        return cmd_vel;
    }

    rclcpp::Node::SharedPtr node_;
    double offset_x_ = 0.0;
    double offset_y_ = 0.0;
    double search_radius_step_ = 0.1;
    double max_search_radius_ = 1.5;
    double sector_angle_ = 0.5;
    int obstacle_threshold_ = 150;
    std::string map_frame_ = "map";
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    bool need_self_save_ = false;
    int consecutive_unsafe_ticks_ = 0;
    int self_save_debounce_ticks_ = 3;

    // 重选点缓存：同目标且仍超阈值时直接复用上次的新点，避免重复圆弧搜索
    double last_judged_target_x_ = 0.0;
    double last_judged_target_y_ = 0.0;
    geometry_msgs::msg::PoseStamped last_selected_goal_;
    bool has_cached_goal_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool self_save_config_loaded_ = false;
    int self_save_stop_threshold_ = SELF_SAVE_STOP_THRESHOLD;
    int self_save_sample_directions_ = SELF_SAVE_SAMPLE_DIRECTIONS;
    double self_save_speed_magnitude_ = SELF_SAVE_SPEED_MAGNITUDE;
    double self_save_sample_radius_ = SELF_SAVE_SAMPLE_RADIUS;
    int self_save_trigger_threshold_ = SELF_SAVE_TRIGGER_THRESHOLD;
};

} // namespace rm2026_decision
