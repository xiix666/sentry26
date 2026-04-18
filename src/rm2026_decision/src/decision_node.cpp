#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// RM2026 接口消息
#include <rm_interfaces/msg/camera_msg.hpp>
#include <rm_interfaces/msg/receive_llc.hpp>
#include <rm_interfaces/msg/nav_msg.hpp>
#include <rm_interfaces/msg/status_msg.hpp>



#include <memory>
#include <string>
#include <filesystem>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <limits>

namespace fs = std::filesystem;

// ===== 配置结构体定义 =====联盟赛nav_to_project_offset_x = 0.75;double nav_to_project_offset_y = 7.0;
struct CoordinateConfig {
    double nav_to_project_offset_x = 1.2;
    double nav_to_project_offset_y = 6.5;
    double project_to_nav_offset_x = -1.2;
    double project_to_nav_offset_y = -6.5;
};

struct BehaviorTreeConfig {
    std::string tree_file = "HighPTree.xml";
    std::string tree_dir = "config";
    double tick_frequency = 10.0;
};

class DecisionNode : public rclcpp::Node
{
public:
    // 队伍颜色枚举
    enum class TeamColor {
        BLUE = 0,  // 蓝方为0
        RED = 1    // 红方为1
    };

public:
    DecisionNode()
        : Node("rm2026_decision_node")
        , current_team_color_(TeamColor::BLUE)  // 默认蓝方
        , global_blackboard_(BT::Blackboard::create())
    {
        declareParameters();
        initializeTFListener();
        initializeCostmapChecker();  // 代价地图检查器初始化
    }

    ~DecisionNode()
    {
        stopExecutionThread();
    }

    void initialize()
    {
        try {
            RCLCPP_INFO(this->get_logger(), "DecisionNode 初始化开始");

            // 阶段1: 参数加载
            loadParameters();

            // 阶段2: 行为树工厂、黑板、区域、代价地图已在构造函数完成 TF/代价地图
            initializeBehaviorTreeFactory();
            initializeAreaPolygons();

            // 阶段3: 行为树加载（执行线程在收到 game_progress==4 比赛开始后由裁判回调启动）
            loadBehaviorTree(bt_config_.tree_file, bt_config_.tree_dir);

            // 阶段4: 通信与定时器
            setupSubscriptions();
            setupStatusTimer();
            setupAreaUpdateTimer();
            setupSelfSaveStatusTimer();
            setupCostmapFetchTimer();

            logStartupInfo();
            RCLCPP_INFO(this->get_logger(), "DecisionNode 初始化完成");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "DecisionNode 初始化失败: %s", e.what());
            throw;
        }
    }

private:
    // ===== 配置结构体 =====
    CoordinateConfig coord_config_;
    BehaviorTreeConfig bt_config_;

    // 阶段1: 参数和配置管理
    void declareParameters()
    {
        this->declare_parameter<std::string>("tree_file", "HighPTree.xml");
        this->declare_parameter<std::string>("tree_directory", "config");
        this->declare_parameter<double>("tick_frequency", 10.0);
        // // 地图与导航坐标系偏置：导航系 -> 本项目系（用于 TF 自身坐标）
        // this->declare_parameter<double>("nav_to_project_offset_x", 0.6);
        // this->declare_parameter<double>("nav_to_project_offset_y", 2.0);
        // // 地图与导航坐标系偏置：本项目系 -> 导航系（用于发送目标点）
        // this->declare_parameter<double>("project_to_nav_offset_x", -0.6);
        // this->declare_parameter<double>("project_to_nav_offset_y", -2.0);
                // 地图与导航坐标系偏置：导航系 -> 本项目系（用于 TF 自身坐标）
        this->declare_parameter<double>("nav_to_project_offset_x", 1.2) ;
        this->declare_parameter<double>("nav_to_project_offset_y", 6.5);
        // 地图与导航坐标系偏置：本项目系 -> 导航系（用于发送目标点）
        this->declare_parameter<double>("project_to_nav_offset_x", -1.2);
        this->declare_parameter<double>("project_to_nav_offset_y", -6.5);
        // team_color 参数已移除，队伍颜色完全由裁判系统消息确定
    }

    void loadParameters()
    {
        bt_config_.tree_file = this->get_parameter("tree_file").as_string();
        bt_config_.tree_dir = this->get_parameter("tree_directory").as_string();
        bt_config_.tick_frequency = this->get_parameter("tick_frequency").as_double();
        coord_config_.nav_to_project_offset_x = this->get_parameter("nav_to_project_offset_x").as_double();
        coord_config_.nav_to_project_offset_y = this->get_parameter("nav_to_project_offset_y").as_double();
        coord_config_.project_to_nav_offset_x = this->get_parameter("project_to_nav_offset_x").as_double();
        coord_config_.project_to_nav_offset_y = this->get_parameter("project_to_nav_offset_y").as_double();
    }

    // 阶段2: 基础设施初始化
    void initializeTFListener()
    {
        // 队伍颜色现在完全由裁判系统消息确定
        // 在裁判系统消息到达前，使用蓝方作为默认值
        current_team_color_ = TeamColor::BLUE;  // 默认蓝方

        // 创建 TF 缓冲区和监听器
        tf_.buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_.listener = std::make_shared<tf2_ros::TransformListener>(*tf_.buffer);

        // 创建定时器，每100ms更新一次位置信息 (10Hz)
        tf_.timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->tfTimerCallback(); });

        RCLCPP_INFO(this->get_logger(), "TF 位置监听器初始化完成，默认队伍: 蓝方，等待裁判系统消息确认队伍信息，查询频率: 10Hz");
    }

    void initializeCostmapChecker()
    {
        // 声明代价地图相关参数（供 Check 树中 CostmapCheckGoal 节点读取）
        this->declare_parameter<std::string>("costmap_service_name", "/global_costmap/get_costmap");
        this->declare_parameter<double>("service_timeout_s", 1.0);
        this->declare_parameter<double>("search_radius_step", 0.1);
        this->declare_parameter<double>("max_search_radius", 1.5);
        this->declare_parameter<double>("sector_angle", 0.5);
        this->declare_parameter<int>("obstacle_threshold", 150);
        // 自救相关参数（CostmapCheckGoal 节点读取）
        this->declare_parameter<int>("self_save_stop_threshold", 150);
        this->declare_parameter<int>("self_save_sample_directions", 18);
        this->declare_parameter<double>("self_save_speed_magnitude", 1.0);
        this->declare_parameter<double>("self_save_sample_radius", 2.0);
        this->declare_parameter<int>("self_save_trigger_threshold", 250);
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_save");
        this->declare_parameter<std::string>("self_save_status_topic", "self_save_status");
        RCLCPP_INFO(this->get_logger(), "代价地图与自救参数已声明，由 Check 树 CostmapCheckGoal 节点使用");
    }

    void initializeBehaviorTreeFactory()
    {
        if (!global_blackboard_) {
            throw std::runtime_error("global_blackboard_ 未创建");
        }
        loadPlugins();
        initializeGlobalBlackboard();
        RCLCPP_INFO(this->get_logger(), "Behavior tree factory initialized");
    }

    void initializeGlobalBlackboard()
    {
        // ========== 自身位置信息 ==========
        global_blackboard_->set("@current_pose_x", 0.75f);
        global_blackboard_->set("@current_pose_y", 7.0f);

        // ========== 状态变量（int/bool）==========
        global_blackboard_->set("@current_area", -1);  // 当前区域ID (int，-1=未知/区域外)
        global_blackboard_->set("@hero_area", -1);
              // 当前区域ID (-1=未知/区域外)
        global_blackboard_->set("@enemy_infantry_nearby", false);

        // ========== 相机数据（敌方位置信息）==========
        const std::array<std::string, 5> vehicle_types = {"hero", "engineer", "infantry1", "infantry2", "sentry"};
        for (const auto& vehicle_type : vehicle_types) {
            std::string base_key = "@enemy_" + vehicle_type;
            global_blackboard_->set(base_key + "_id", 0);
            global_blackboard_->set(base_key + "_x", 0.0f);
            global_blackboard_->set(base_key + "_y", 0.0f);
            global_blackboard_->set(base_key + "_speed_x", 0.0f);
            global_blackboard_->set(base_key + "_speed_y", 0.0f);
        }

        // 初始化敌人检测状态标志
        global_blackboard_->set("@hero_detected", 0);
        global_blackboard_->set("@engineer_detected", 0);
        global_blackboard_->set("@infantry1_detected", 0);
        global_blackboard_->set("@infantry2_detected", 0);
        global_blackboard_->set("@sentry_enemy_detected", 0);
        global_blackboard_->set("@enemy_count", 0);

        // 初始化敌人距离（-1表示未检测到）
        global_blackboard_->set("@enemy_hero_distance", -1.0f);
        global_blackboard_->set("@enemy_engineer_distance", -1.0f);
        global_blackboard_->set("@enemy_infantry1_distance", -1.0f);
        global_blackboard_->set("@enemy_infantry2_distance", -1.0f);
        global_blackboard_->set("@enemy_sentry_distance", -1.0f);
        global_blackboard_->set("@enemy_distance_min", -1.0f);  // 最近敌人距离

        // ========== 电控数据（游戏状态）==========
        global_blackboard_->set("@game_progress", 1);          // 游戏进度 (0=未开始)
        global_blackboard_->set("@stage_remain_time", 300);      // 阶段剩余时间
        global_blackboard_->set("@red_blue", 0);               // 队伍颜色 (0=蓝方，与系统默认一致)
        global_blackboard_->set("@bullets_allowance", 300);      // 子弹限额
        // global_blackboard_->set("@chassis_power", 0.0f);       // 底盘功率
        global_blackboard_->set("@center_status", 0);          // 中心状态
        global_blackboard_->set("@remain_gold", 0);            // 剩余金币

        // ========== 自救状态（0=非自救，1=自救中，由 Check 树 CostmapCheckGoal 写，本节点定时发布）==========
        global_blackboard_->set("@self_save_status", 0);

        // ========== 自救速度（仅自救时由 CostmapCheckGoal 写，本节点在 self_save_status==1 时 10Hz 发布）==========
        geometry_msgs::msg::Twist cmd_vel_zero;
        cmd_vel_zero.linear.x = cmd_vel_zero.linear.y = cmd_vel_zero.linear.z = 0.0;
        cmd_vel_zero.angular.x = cmd_vel_zero.angular.y = cmd_vel_zero.angular.z = 0.0;
        global_blackboard_->set("@cmd_vel_save", cmd_vel_zero);

        // ========== 己方血量 ==========
        global_blackboard_->set("@hero_hp", 400);              // 英雄血量 (满血)
        global_blackboard_->set("@infantry_hp", 300);          // 步兵血量 (满血)
        global_blackboard_->set("@sentry_hp", 400);            // 哨兵血量 (满血)

        // 补给点
        geometry_msgs::msg::PoseStamped supply_pose;
        supply_pose.header.frame_id = "map";
        supply_pose.pose.position.x = 1.0;  // (1500/2)/1000 = 0.75m
        supply_pose.pose.position.y = 6.8;   // ((6000+8000)/2)/1000 = 7.0m
        supply_pose.pose.position.z = 0.0;
        global_blackboard_->set("@supply_point_location", supply_pose);

        // 补给点（用于测试）
        // geometry_msgs::msg::PoseStamped supply_pose;
        // supply_pose.header.frame_id = "map";
        // supply_pose.pose.position.x = 1.85;  // (1500/2)/1000 = 0.75m
        // supply_pose.pose.position.y = 2.7;   // ((6000+8000)/2)/1000 = 7.0m
        // supply_pose.pose.position.z = 0.0;
        // global_blackboard_->set("@supply_point_location", supply_pose);

        // 中心增益点（区域4中心）
        geometry_msgs::msg::PoseStamped center_pose;
        center_pose.header.frame_id = "map";
        center_pose.pose.position.x = 6.0;   // ((4500+7500)/2)/1000 = 6.0m
        center_pose.pose.position.y = 4.0;   // ((2500+5500)/2)/1000 = 4.0m
        center_pose.pose.position.z = 0.0;
        global_blackboard_->set("@center_capture_point", center_pose);

        // 队友位置（仅使用 x/y 坐标，由裁判系统消息更新）
        global_blackboard_->set("@teammate_hero_location_x", 0.0f);
        global_blackboard_->set("@teammate_hero_location_y", 0.0f);
        global_blackboard_->set("@teammate_infantry_location_x", 0.0f);
        global_blackboard_->set("@teammate_infantry_location_y", 0.0f);

        // 英雄跟踪相关变量
        global_blackboard_->set("@hero_follow_center_x", 0.0f);     // 英雄跟踪圆心X坐标
        global_blackboard_->set("@hero_follow_center_y", 0.0f);     // 英雄跟踪圆心Y坐标

        // 区域一巡逻点（1250,5700），（840,3900），（1700,3900）
        geometry_msgs::msg::PoseStamped area1_p1;
        area1_p1.header.frame_id = "map";
        area1_p1.pose.position.x = 1.25;
        area1_p1.pose.position.y = 5.7;
        area1_p1.pose.position.z = 0.0;
        global_blackboard_->set("@area1_point1", area1_p1);

        geometry_msgs::msg::PoseStamped area1_p2;
        area1_p2.header.frame_id = "map";
        area1_p2.pose.position.x = 0.84;
        area1_p2.pose.position.y = 3.9;
        area1_p2.pose.position.z = 0.0;
        global_blackboard_->set("@area1_point2", area1_p2);

        geometry_msgs::msg::PoseStamped area1_p3;
        area1_p3.header.frame_id = "map";
        area1_p3.pose.position.x = 1.7;
        area1_p3.pose.position.y = 3.9;
        area1_p3.pose.position.z = 0.0;
        global_blackboard_->set("@area1_point3", area1_p3);

        //TestPatrol3用于测试的坐标
        // geometry_msgs::msg::PoseStamped area1_p1;
        // area1_p1.header.frame_id = "map";
        // area1_p1.pose.position.x = 4.0;
        // area1_p1.pose.position.y = 2.6;
        // area1_p1.pose.position.z = 0.0;
        // global_blackboard_->set("@area1_point1", area1_p1);

        // geometry_msgs::msg::PoseStamped area1_p2;
        // area1_p2.header.frame_id = "map";
        // area1_p2.pose.position.x = 6.7;
        // area1_p2.pose.position.y = 3.1;
        // area1_p2.pose.position.z = 0.0;
        // global_blackboard_->set("@area1_point2", area1_p2);

        // geometry_msgs::msg::PoseStamped area1_p3;
        // area1_p3.header.frame_id = "map";
        // area1_p3.pose.position.x = 6.7;
        // area1_p3.pose.position.y = 1.6;
        // area1_p3.pose.position.z = 0.0;
        // global_blackboard_->set("@area1_point3", area1_p3);

        geometry_msgs::msg::PoseStamped TestQif_point1;
        TestQif_point1.header.frame_id = "map";
        TestQif_point1.pose.position.x = 0.0;
        TestQif_point1.pose.position.y = 0.0;
        TestQif_point1.pose.position.z = 0.0;
        global_blackboard_->set("@TestQif_point1", TestQif_point1);

        geometry_msgs::msg::PoseStamped TestQif_point2;
        TestQif_point2.header.frame_id = "map";
        TestQif_point2.pose.position.x = 3.0;
        TestQif_point2.pose.position.y = 0.0;
        TestQif_point2.pose.position.z = 0.0;
        global_blackboard_->set("@TestQif_point2", TestQif_point2);

        // 区域二黄色区域目标点
        geometry_msgs::msg::PoseStamped area2_yellow_pose;
        area2_yellow_pose.header.frame_id = "map";
        area2_yellow_pose.pose.position.x = 3.7;
        area2_yellow_pose.pose.position.y = 2.40;
        area2_yellow_pose.pose.position.z = 0.0;
        global_blackboard_->set("@area2_yellow_point", area2_yellow_pose);

        // 区域三巡逻点
        geometry_msgs::msg::PoseStamped area3_p1;
        area3_p1.header.frame_id = "map";
        area3_p1.pose.position.x = 4.8;
        area3_p1.pose.position.y = 1.5;
        area3_p1.pose.position.z = 0.0;
        global_blackboard_->set("@area3_point1", area3_p1);

        geometry_msgs::msg::PoseStamped area3_p2;
        area3_p2.header.frame_id = "map";
        area3_p2.pose.position.x = 6.9;
        area3_p2.pose.position.y = 2.3;
        area3_p2.pose.position.z = 0.0;
        global_blackboard_->set("@area3_point2", area3_p2);

        geometry_msgs::msg::PoseStamped area3_p3;
        area3_p3.header.frame_id = "map";
        area3_p3.pose.position.x = 6.9;
        area3_p3.pose.position.y = 0.8;
        area3_p3.pose.position.z = 0.0;
        global_blackboard_->set("@area3_point3", area3_p3);

        // 区域四黄红点
        geometry_msgs::msg::PoseStamped area4_red_patrol;
        area4_red_patrol.header.frame_id = "map";
        area4_red_patrol.pose.position.x = 6.9;
        area4_red_patrol.pose.position.y = 3.4;
        area4_red_patrol.pose.position.z = 0.0;
        global_blackboard_->set("@area4_red_patrol", area4_red_patrol);

        geometry_msgs::msg::PoseStamped area4_yellow_patrol;
        area4_yellow_patrol.header.frame_id = "map";
        area4_yellow_patrol.pose.position.x = 5.1;
        area4_yellow_patrol.pose.position.y = 4.6;
        area4_yellow_patrol.pose.position.z = 0.0;
        global_blackboard_->set("@area4_yellow_patrol", area4_yellow_patrol);
        // 开局到点标志位：0=未到达，1=已到达
        global_blackboard_->set("@area4_yellow_patrol_reached", 0);

        // 前往区域指令点
        geometry_msgs::msg::PoseStamped goto_area1;
        goto_area1.header.frame_id = "map";
        goto_area1.pose.position.x = 1.3;
        goto_area1.pose.position.y = 4.8;
        goto_area1.pose.position.z = 0.0;
        global_blackboard_->set("@goto_area1", goto_area1);

        geometry_msgs::msg::PoseStamped goto_area2;
        goto_area2.header.frame_id = "map";
        goto_area2.pose.position.x = 2.5;
        goto_area2.pose.position.y = 1.4;
        goto_area2.pose.position.z = 0.0;
        global_blackboard_->set("@goto_area2", goto_area2);

        geometry_msgs::msg::PoseStamped goto_area3;
        goto_area3.header.frame_id = "map";
        goto_area3.pose.position.x = 5.0;
        goto_area3.pose.position.y = 1.5;
        goto_area3.pose.position.z = 0.0;
        global_blackboard_->set("@goto_area3", goto_area3);

        geometry_msgs::msg::PoseStamped goto_area4;
        goto_area4.header.frame_id = "map";
        goto_area4.pose.position.x = 6.8;
        goto_area4.pose.position.y = 3.2;
        goto_area4.pose.position.z = 0.0;
        global_blackboard_->set("@goto_area4", goto_area4);


        // geometry_msgs::msg::PoseStamped goto_area1;
        // goto_area1.header.frame_id = "map";
        // goto_area1.pose.position.x = 6.9;
        // goto_area1.pose.position.y = 4.55;
        // goto_area1.pose.position.z = 0.0;
        // global_blackboard_->set("@goto_area1", goto_area1);

        // geometry_msgs::msg::PoseStamped goto_area2;
        // goto_area2.header.frame_id = "map";
        // goto_area2.pose.position.x = 6.9;
        // goto_area2.pose.position.y = 4.55;
        // goto_area2.pose.position.z = 0.0;
        // global_blackboard_->set("@goto_area2", goto_area2);

        // geometry_msgs::msg::PoseStamped goto_area3;
        // goto_area3.header.frame_id = "map";
        // goto_area3.pose.position.x = 6.9;
        // goto_area3.pose.position.y = 4.55;
        // goto_area3.pose.position.z = 0.0;
        // global_blackboard_->set("@goto_area3", goto_area3);

        // geometry_msgs::msg::PoseStamped goto_area4;
        // goto_area4.header.frame_id = "map";
        // goto_area4.pose.position.x = 6.9;
        // goto_area4.pose.position.y = 4.55;
        // goto_area4.pose.position.z = 0.0;
        // global_blackboard_->set("@goto_area4", goto_area4);

        // 撤退到区域数减小方向（暂时使用区域二黄色区域点）
        geometry_msgs::msg::PoseStamped lower_area_pose;
        lower_area_pose.header.frame_id = "map";
        lower_area_pose.pose.position.x = 4.0;
        lower_area_pose.pose.position.y = 2.75;
        lower_area_pose.pose.position.z = 0.0;
        global_blackboard_->set("@lower_area_location", lower_area_pose);

        // 辅助变量
        global_blackboard_->set("@current_patrol_point", 0);

        // 导航目标点
        geometry_msgs::msg::PoseStamped default_nav_goal;
        default_nav_goal.header.frame_id = "map";
        default_nav_goal.header.stamp = this->get_clock()->now();
        default_nav_goal.pose.position.x = 0.75;
        default_nav_goal.pose.position.y = 7.0;
        default_nav_goal.pose.position.z = 0.0;
        //default_nav_goal.pose.orientation.w = 1.0;  // 默认朝向
        global_blackboard_->set("@nav_goal", default_nav_goal);  // 导航目标位置
        global_blackboard_->set("@nav_goal_requested", false);  // 是否有导航目标请求

        RCLCPP_INFO(this->get_logger(), "global blackboard initialized,using @");
    }

    // 加载行为节点插件（行为节点库的初始化函数（BT 4.8 插件加载时查找的符号名））
    void loadPlugins()
    {
        const std::string prefix = ament_index_cpp::get_package_prefix("rm2026_decision");
        fs::path plugin_dir = fs::path(prefix) / "lib";

        const std::vector<std::string> plugins = {
            "libbehavior_nodes.so",
        };

        for (const auto& plugin : plugins)
        {
            fs::path plugin_path = plugin_dir / plugin;
            if (!fs::exists(plugin_path))
            {
                throw std::runtime_error("behavior node plugin not found: " + plugin_path.string());
            }
            bt_.factory.registerFromPlugin(plugin_path.string());
            RCLCPP_INFO(this->get_logger(), "Loaded plugin: %s", plugin_path.string().c_str());
        } 
    }

    // 阶段3: 通信系统设置
    void setupSubscriptions()
    {
        // 相机消息订阅 - 敌方位置和速度信息
        comm_.camera = this->create_subscription<rm_interfaces::msg::CameraMsg>(
            "/gimbal_camera_topic", 10,
            [this](const rm_interfaces::msg::CameraMsg::SharedPtr msg) {
                updateCameraData(msg);
            });

        // 裁判系统消息订阅 - 比赛状态和己方信息
        comm_.referee = this->create_subscription<rm_interfaces::msg::ReceiveLLC>(
            "/receiveLLC_pack", 10,
            [this](const rm_interfaces::msg::ReceiveLLC::SharedPtr msg) {
                updateRefereeData(msg);
            });

        // 导航目标发布者 - 统一管理所有导航节点的目标发布
        comm_.nav_goal = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);

        // NavMsg 发布者 - 发送导航禁用状态
        comm_.nav_msg = this->create_publisher<rm_interfaces::msg::NavMsg>(
            "/nav_pack", 10);

        // StatusMsg 发布者 - 发送哨兵姿态指令
        comm_.status_msg = this->create_publisher<rm_interfaces::msg::StatusMsg>(
            "/status_pack", 10);

        // 自救状态发布者 - 定时从黑板 @self_save_status 读取并发布（0=非自救，1=自救中）
        std::string self_save_topic = "self_save_status";
        if (this->has_parameter("self_save_status_topic")) {
            self_save_topic = this->get_parameter("self_save_status_topic").as_string();
        }
        comm_.self_save_status = this->create_publisher<std_msgs::msg::Int32>(self_save_topic, 10);

        // 自救速度发布者 - 仅当 self_save_status==1 时 10Hz 从黑板 @cmd_vel_save 读取并发布
        std::string cmd_vel_topic = "cmd_vel_save";
        if (this->has_parameter("cmd_vel_topic")) {
            cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        }
        comm_.cmd_vel_save = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Message subscriptions initialized: camera data, referee system data");
    }

    void setupStatusTimer()
    {
        double status_period = 1.0;
        timers_.status = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(status_period)),
            std::bind(&DecisionNode::publishStatusCallback, this));
    }

    void setupAreaUpdateTimer()
    {
        // 创建5Hz定时器用于更新当前区域
        timers_.area_update = this->create_wall_timer(
            std::chrono::milliseconds(200),  // 5Hz = 200ms
            [this]() { this->areaUpdateTimerCallback(); });
        RCLCPP_INFO(this->get_logger(), "Area update timer initialized, frequency: 5Hz");
    }

    void setupSelfSaveStatusTimer()
    {
        // 定时从黑板读取 @self_save_status 并发布（10Hz）
        timers_.self_save_status = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->selfSaveStatusTimerCallback(); });
        RCLCPP_INFO(this->get_logger(), "Self-save status publish timer initialized, 10Hz");
    }

    void setupCostmapFetchTimer()
    {
        std::string service_name = this->get_parameter("costmap_service_name").as_string();
        comm_.costmap_client = this->create_client<nav2_msgs::srv::GetCostmap>(service_name);
        timers_.costmap_fetch = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->costmapFetchTimerCallback(); });
        RCLCPP_INFO(this->get_logger(), "Costmap fetch timer initialized, 1Hz, service: %s", service_name.c_str());
    }

    void costmapFetchTimerCallback()
    {
        if (!comm_.costmap_client || !global_blackboard_) return;
        if (!comm_.costmap_client->service_is_ready()) return;
        auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        comm_.costmap_client->async_send_request(
            request,
            [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result && global_blackboard_) {
                        global_blackboard_->set("@costmap", result->map);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_DEBUG(this->get_logger(), "Costmap fetch failed: %s", e.what());
                }
            });
    }

    void selfSaveStatusTimerCallback()
    {
        if (!global_blackboard_) return;
        try {
            int value = global_blackboard_->get<int>("@self_save_status");
            if (comm_.self_save_status) {
                std_msgs::msg::Int32 msg;
                msg.data = value;
                comm_.self_save_status->publish(msg);
            }
            // 仅自救状态(1)时 10Hz 发布速度；非自救时发零速
            if (comm_.cmd_vel_save) {
                if (value == 1) {
                    geometry_msgs::msg::Twist cmd = global_blackboard_->get<geometry_msgs::msg::Twist>("@cmd_vel_save");
                    comm_.cmd_vel_save->publish(cmd);
                } else {
                    geometry_msgs::msg::Twist zero;
                    zero.linear.x = zero.linear.y = zero.linear.z = 0.0;
                    zero.angular.x = zero.angular.y = zero.angular.z = 0.0;
                    comm_.cmd_vel_save->publish(zero);
                }
            }
        } catch (const std::exception&) {
            // 黑板尚未写入时忽略
        }
    }

    // 阶段4: 行为树生命周期
    void loadBehaviorTree(const std::string& tree_file, const std::string& tree_dir)
    {
        if (!global_blackboard_) {
            throw std::runtime_error("loadBehaviorTree: global_blackboard_ 未就绪");
        }
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("rm2026_decision");
            fs::path tree_dir_path = fs::path(package_share_dir) / tree_dir;
            fs::path tree_path = tree_dir_path / tree_file;

            if (!fs::exists(tree_path)) {
                throw std::runtime_error("tree file not found: " + tree_path.string());
            }

            // 使用全局黑板，确保所有信息都在全局黑板中
            global_blackboard_->set("@ros_node", shared_from_this());

            // 先按依赖顺序注册子树：Area0Tree..Area5Tree → AreaMain，再加载主树 HighP
            const std::vector<std::string> subtree_files = {
                "Area0Tree.xml", "Area1Tree.xml", "Area2Tree.xml",
                "Area3Tree.xml", "Area4Tree.xml", "Area5Tree.xml",
                "AreaMain.xml",
                "FollowHeroTest.xml", "FollowHeroArea.xml",
                "CheckTree.xml","TestPatrol3.xml",
                "TestQif.xml"
            };
            for (const auto& sub_file : subtree_files) {
                fs::path sub_path = tree_dir_path / sub_file;
                if (fs::exists(sub_path)) {
                    bt_.factory.registerBehaviorTreeFromFile(sub_path.string());
                    RCLCPP_INFO(this->get_logger(), "Registered subtree file: %s", sub_path.string().c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "subtree file not found, skip: %s", sub_path.string().c_str());
                }
            }

            // 主树文件也通过 register 注册，再用 createTree 按名创建（避免混用 createTreeFromFile 导致找不到子树）
            bt_.factory.registerBehaviorTreeFromFile(tree_path.string());
            RCLCPP_INFO(this->get_logger(), "registered main tree file: %s", tree_path.string().c_str());

            const std::string main_tree_id = "HighP";
            // const std::string main_tree_id = "TestQifTree";
            const std::string main_tree_check = "CheckTree";
            const std::string main_tree_monitor = "Area0Tree";
            
            bt_.tree = std::make_unique<BT::Tree>(bt_.factory.createTree(main_tree_id, global_blackboard_));
            bt_.tree_Check = std::make_unique<BT::Tree>(bt_.factory.createTree(main_tree_check, global_blackboard_));
            bt_.tree_monitor = std::make_unique<BT::Tree>(bt_.factory.createTree(main_tree_monitor, global_blackboard_));

            // 初始化 Groot2 Publisher，用于和 Groot2 进行实时监控通信，这里使用 1667 (server-groot2)
            bt_.groot2_publisher = std::make_unique<BT::Groot2Publisher>(*bt_.tree_monitor, 1667);
            //startTreeExecutionThread();
            RCLCPP_INFO(this->get_logger(), "Behavior Tree Loaded Successfully: %s (ID=%s)", tree_path.string().c_str(), main_tree_id.c_str());
            RCLCPP_INFO(this->get_logger(), "Use global blackboard and enable Groot2 monitoring");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "failed to load behavior tree: %s", e.what());
            throw;
        }
    }

    void startTreeExecutionThread()
    {
        if (!bt_.tree) {
            throw std::runtime_error("startTreeExecutionThread: 行为树未加载");
        }
        if (bt_.execution_thread) {
            //RCLCPP_WARN(this->get_logger(), "Execution thread is already running");
            return;
        }

        bt_.stop_execution = false;
        bt_.execution_thread = std::make_unique<std::thread>(
            &DecisionNode::treeExecutionLoop, this);
        RCLCPP_INFO(this->get_logger(), "behavior tree execution thread started");
    }

    void stopExecutionThread()
    {
        if (!bt_.execution_thread) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stopping behavior tree execution thread...");
        bt_.stop_execution = true;

        if (bt_.execution_thread->joinable()) {
            bt_.execution_thread->join();
        }

        bt_.execution_thread.reset();
        RCLCPP_INFO(this->get_logger(), "Behavior tree execution thread stopped");
    }

    void treeExecutionLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Behavior tree execution loop started, target frequency: %.1f Hz", bt_config_.tick_frequency);

        while (!bt_.stop_execution && rclcpp::ok()) {
            if (!bt_.tree) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // 仅在比赛进行中 (game_progress==4) 执行；非 4 时退出循环
            {
                std::lock_guard<std::mutex> lock(game_state_mutex_);
                try {
                    auto gp = global_blackboard_->get<int>("@game_progress");
                    if (gp != 4) {
                        RCLCPP_INFO(this->get_logger(), "Game progress=%d (not 4), behavior tree execution loop exiting", gp);
                        break;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error reading game_progress: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }

            auto tick_start = std::chrono::steady_clock::now();

            if (global_blackboard_ && comm_.nav_goal) {
                geometry_msgs::msg::PoseStamped goal_to_publish;
                try {
                    std::lock_guard<std::mutex> lock(game_state_mutex_);
                    auto nav_goal_requested = global_blackboard_->get<bool>("@nav_goal_requested");
                    if (nav_goal_requested) {
                        auto nav_goal = global_blackboard_->get<geometry_msgs::msg::PoseStamped>("@nav_goal");
                        goal_to_publish = nav_goal;
                        goal_to_publish.pose.position.x = nav_goal.pose.position.x + coord_config_.project_to_nav_offset_x;
                        goal_to_publish.pose.position.y = nav_goal.pose.position.y + coord_config_.project_to_nav_offset_y;
                        goal_to_publish.header.stamp = this->get_clock()->now();
                        comm_.nav_goal->publish(goal_to_publish);
                        RCLCPP_INFO(this->get_logger(), "Published navigation goal: (%.2f, %.2f)",
                            nav_goal.pose.position.x, nav_goal.pose.position.y);
                        global_blackboard_->set("@nav_goal_requested", false);
                    }
                            
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error reading navigation/game state: %s", e.what());
                }
            }

            try {
                BT::NodeStatus status = bt_.tree->tickOnce();

                if (status == BT::NodeStatus::RUNNING) {
                    bt_.tree_running = true;
                } else {
                    bt_.tree_running = false;
                }

                if (BT::isStatusCompleted(status)) {
                    RCLCPP_INFO(this->get_logger(), "over the behavior tree, status: %s",
                               status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Behavior tree execution exception: %s", e.what());
                bt_.tree_running = false;
            }

            // Check 树：与主树同频
            if (bt_.tree_Check) {
                try {
                    bt_.tree_Check->tickOnce();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Check behavior tree execution exception: %s", e.what());
                }
            }

            auto tick_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tick_end - tick_start);
            auto target_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / bt_config_.tick_frequency));

            if (elapsed < target_interval) {
                auto sleep_duration = target_interval - elapsed;
                std::this_thread::sleep_for(sleep_duration);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Behavior tree execution time(%ldms) exceeds target interval(%ldms), may affect execution frequency",
                    elapsed.count(), target_interval.count());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Behavior tree execution loop ended");
    }

    // 阶段5: 运行时回调
    void updateCameraData(const rm_interfaces::msg::CameraMsg::SharedPtr msg)
    {
        if (!global_blackboard_) {
            return;
        }

        // 敌人 pose_x/pose_y 来自 base_link 坐标系（相对机器人）。
        // 我们需要把它转换到 map(nav) 坐标系，再加偏置变成项目坐标系存入黑板。
        bool has_robot_project_pose = false;
        float robot_project_x = 0.0f;
        float robot_project_y = 0.0f;
        float robot_nav_x = 0.0f;   // map(nav) 下机器人位置
        float robot_nav_y = 0.0f;
        try {
            robot_project_x = global_blackboard_->get<float>("@current_pose_x");
            robot_project_y = global_blackboard_->get<float>("@current_pose_y");
            has_robot_project_pose = true;
            robot_nav_x = robot_project_x - static_cast<float>(coord_config_.nav_to_project_offset_x);
            robot_nav_y = robot_project_y - static_cast<float>(coord_config_.nav_to_project_offset_y);
        } catch (...) {
            // 若当前位姿尚未写入黑板，则保持旧逻辑：直接把 base_link 坐标写入（避免崩溃）
            has_robot_project_pose = false;
        }

        // 车辆类型定义：英雄、工程、步兵1、步兵2、哨兵(根)
        const std::array<std::string, 5> vehicle_types = {"hero", "engineer", "infantry1", "infantry2", "sentry"};

        // 重置敌人检测状态标志（每次收到新消息时都要重置）
        for (const auto& vehicle_type : vehicle_types) {
            std::string detected_key = "@" + vehicle_type + "_detected";
            if (vehicle_type == "sentry") {
                detected_key = "@sentry_enemy_detected";
            }
            global_blackboard_->set(detected_key, 0);
        }

        // 相机数据指针数组
        const std::array<const std::array<int32_t, 5>*, 4> id_arrays = {
            &msg->id_1, &msg->id_2, &msg->id_3, &msg->id_4
        };
        const std::array<const std::array<float, 5>*, 4> pose_x_arrays = {
            &msg->enemy_pose_1_x, &msg->enemy_pose_2_x, &msg->enemy_pose_3_x, &msg->enemy_pose_4_x
        };
        const std::array<const std::array<float, 5>*, 4> pose_y_arrays = {
            &msg->enemy_pose_1_y, &msg->enemy_pose_2_y, &msg->enemy_pose_3_y, &msg->enemy_pose_4_y
        };

        int total_enemy_count = 0;

        // 处理每个相机的检测结果
        for (size_t camera_idx = 0; camera_idx < id_arrays.size(); ++camera_idx) {
            const auto& id_array = *id_arrays[camera_idx];
            const auto& pose_x_array = *pose_x_arrays[camera_idx];
            const auto& pose_y_array = *pose_y_arrays[camera_idx];

            // 处理每种车辆类型 (5种：英雄、工程、步兵1、步兵2、哨兵)
            for (size_t vehicle_idx = 0; vehicle_idx < vehicle_types.size(); ++vehicle_idx) {
                int enemy_id = id_array[vehicle_idx];

                if (enemy_id != 0) {  // 检测到敌人
                    const std::string& vehicle_type = vehicle_types[vehicle_idx];
                    float enemy_x_base = pose_x_array[vehicle_idx];
                    float enemy_y_base = pose_y_array[vehicle_idx];

                    // 计算距离：sqrt(x^2 + y^2)
                    float enemy_distance = std::sqrt(enemy_x_base * enemy_x_base + enemy_y_base * enemy_y_base);

                    // 将检测结果存储到黑板，格式：@enemy_{type}_distance_{camera_idx}
                    // 存储为距离值
                    std::string distance_key = "@enemy_" + vehicle_type + "_distance_" + std::to_string(camera_idx);
                    global_blackboard_->set(distance_key, enemy_distance);

                    // 标记该类型敌人被至少一个相机检测到
                    std::string detected_key = "@" + vehicle_type + "_detected";
                    if (vehicle_type == "sentry") {
                        detected_key = "@sentry_enemy_detected";
                    }
                    global_blackboard_->set(detected_key, 1);

                    // 设置ID（总是更新最新的有效ID）
                    std::string id_key = "@enemy_" + vehicle_type + "_id";
                    global_blackboard_->set(id_key, enemy_id);

                    // 设置位置信息
                    std::string x_key = "@enemy_" + vehicle_type + "_x";
                    std::string y_key = "@enemy_" + vehicle_type + "_y";
                    // 敌人所在区域 id：0/1/.../8，区域外为 -1
                    std::string area_key = "@enemy_" + vehicle_type + "_area";
                    // base_link -> map(nav)（平移）-> project（加偏置）
                    if (has_robot_project_pose) {
                        float enemy_nav_x = robot_nav_x + enemy_x_base;
                        float enemy_nav_y = robot_nav_y + enemy_y_base;
                        float enemy_project_x = enemy_nav_x + static_cast<float>(coord_config_.nav_to_project_offset_x);
                        float enemy_project_y = enemy_nav_y + static_cast<float>(coord_config_.nav_to_project_offset_y);
                        global_blackboard_->set(x_key, enemy_project_x);
                        global_blackboard_->set(y_key, enemy_project_y);

                        int enemy_area_id = determineCurrentArea(enemy_project_x, enemy_project_y);
                        global_blackboard_->set(area_key, enemy_area_id);

                        RCLCPP_INFO(this->get_logger(),
                                    "Enemy %s project pos=(%.2f,%.2f),area=%d",
                                    vehicle_type.c_str(),
                                    enemy_project_x, enemy_project_y,
                                    enemy_area_id);
                    } else {
                        // fallback：写入 base_link 坐标，至少不阻断流程
                        global_blackboard_->set(x_key, enemy_x_base);
                        global_blackboard_->set(y_key, enemy_y_base);
                        global_blackboard_->set(area_key, -1);
                    }

                    // 设置速度信息（如果有的话）
                    if (!msg->speed_x.empty() && !msg->speed_y.empty() &&
                        vehicle_idx < msg->speed_x.size() && vehicle_idx < msg->speed_y.size()) {
                        std::string speed_x_key = "@enemy_" + vehicle_type + "_speed_x";
                        std::string speed_y_key = "@enemy_" + vehicle_type + "_speed_y";
                        // 总是更新最新的速度信息
                        global_blackboard_->set(speed_x_key, msg->speed_x[vehicle_idx]);
                        global_blackboard_->set(speed_y_key, msg->speed_y[vehicle_idx]);
                    }
                }
            }
        }

        // 统计检测到的敌人总数
        for (const auto& vehicle_type : vehicle_types) {
            std::string detected_key = "@" + vehicle_type + "_detected";
            if (vehicle_type == "sentry") {
                detected_key = "@sentry_enemy_detected";
            }
            try {
                if (global_blackboard_->get<int>(detected_key) == 1) {
                    total_enemy_count++;
                }
            } catch (...) {
                // 忽略异常
            }
        }

        // 更新敌人统计信息到黑板
        global_blackboard_->set("@enemy_count", total_enemy_count);

        // 构建统一日志输出
        std::stringstream debug_msg;
        debug_msg << "Detected enemies:\n";

        RCLCPP_DEBUG(this->get_logger(), "%s", debug_msg.str().c_str());
    }

    void updateRefereeData(const rm_interfaces::msg::ReceiveLLC::SharedPtr msg)
    {
        if (!global_blackboard_) {
            return;
        }

        // 更新比赛状态信息（game_progress 与行为树/代价地图回调并发访问，需加锁）
        {
            std::lock_guard<std::mutex> lock(game_state_mutex_);
            global_blackboard_->set("@game_progress", msg->game_progress);
        }
        
        // 比赛开始 (game_progress==4) 时启动行为树执行线程，非 4 时停止
        //msg->game_progress == 4
        if (msg->game_progress == 4) {
            startTreeExecutionThread();
        } else {
            global_blackboard_->set("@area4_yellow_patrol_reached", 0);
            stopExecutionThread();
        }
    
        
        global_blackboard_->set("@stage_remain_time", msg->stage_remain_time);
        global_blackboard_->set("@red_blue", msg->red_blue);

        // 更新己方血量信息
        global_blackboard_->set("@hero_hp", msg->self_hero_hp);
        global_blackboard_->set("@infantry_hp", msg->self_infantry_hp);
        global_blackboard_->set("@sentry_hp", msg->self_sentry_hp);

        // 更新弹药和功率信息
        global_blackboard_->set("@bullets_allowance", msg->bullets_allowance);
        // global_blackboard_->set("@chassis_power", static_cast<float>(msg->chassis_power));

        // 更新联盟赛信息
        global_blackboard_->set("@center_status", msg->center_status);
        global_blackboard_->set("@remain_gold", msg->remain_gold);

        // 更新队友位置（仅 x/y 坐标）
        global_blackboard_->set("@teammate_hero_location_x", msg->hero_pose_x);
        global_blackboard_->set("@teammate_hero_location_y", msg->hero_pose_y);
        global_blackboard_->set("@teammate_infantry_location_x", msg->infantry_pose_x);
        global_blackboard_->set("@teammate_infantry_location_y", msg->infantry_pose_y);

        // 更新英雄所在区域
        updateHeroArea(msg->hero_pose_x, msg->hero_pose_y);

        // 获取队伍颜色
        TeamColor new_team_color = (msg->red_blue == 0) ? TeamColor::BLUE : TeamColor::RED;
        bool team_color_changed = (new_team_color != current_team_color_);

        if (team_color_changed) {
            // 队伍颜色，记录日志
            RCLCPP_INFO(this->get_logger(), "队伍颜色: %s → %s",
                       current_team_color_ == TeamColor::RED ? "红方" : "蓝方",
                       new_team_color == TeamColor::RED ? "红方" : "蓝方");
            current_team_color_ = new_team_color;
        }

        // 第一次收到消息时，输出 INFO 级别日志确认更新成功
        if (!referee_data_received_) {
            referee_data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Referee system data updated successfully: team=%s, game progress=%d, self HP=%d/%d/%d, "
                        "ammo allowance=%d, center status=%d, remaining gold=%d",
                        new_team_color == TeamColor::RED ? "红方" : "蓝方",
                        msg->game_progress, msg->self_hero_hp, msg->self_infantry_hp, msg->self_sentry_hp,
                        msg->bullets_allowance, msg->center_status, msg->remain_gold);
        }

        RCLCPP_DEBUG(this->get_logger(), "Referee system data updated: team=%s, game progress=%d, self HP=%d/%d/%d, "
                    "teammate hero location=(%.2f,%.2f), infantry location=(%.2f,%.2f)",
                    current_team_color_ == TeamColor::RED ? "红方" : "蓝方",
                    msg->game_progress, msg->self_hero_hp, msg->self_infantry_hp, msg->self_sentry_hp,
                    msg->hero_pose_x, msg->hero_pose_y, msg->infantry_pose_x, msg->infantry_pose_y);
    }

    void tfTimerCallback()
    {
        std::string target_frame = "map";
        std::string source_frame = "base_link";

        try {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_.buffer->lookupTransform(target_frame, source_frame, tf2::TimePoint());

            // 从导航坐标系转换到本项目坐标系（加偏置）
            double x_nav = transformStamped.transform.translation.x;
            double y_nav = transformStamped.transform.translation.y;
            current_pose_.x = x_nav + coord_config_.nav_to_project_offset_x;
            current_pose_.y = y_nav + coord_config_.nav_to_project_offset_y;

            // 将位置信息写入全局黑板，供行为树节点使用
            if (global_blackboard_) {
                global_blackboard_->set("@current_pose_x", static_cast<float>(current_pose_.x));
                global_blackboard_->set("@current_pose_y", static_cast<float>(current_pose_.y));
            }

        } catch (const tf2::TransformException &ex) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to get transform: %s", ex.what());
        }
    }

    // 区域更新定时器回调函数（5Hz）
    void areaUpdateTimerCallback()
    {
        if (!global_blackboard_) {
            RCLCPP_DEBUG(this->get_logger(), "global_blackboard_ is null");
            return;
        }
        // 获取当前机器人位置
        try {
            auto current_x = global_blackboard_->get<float>("@current_pose_x");
            auto current_y = global_blackboard_->get<float>("@current_pose_y");
            updateCurrentArea(current_x, current_y);
        } catch (const std::exception& e) {
            // 位置信息可能还未更新，忽略
            RCLCPP_DEBUG(this->get_logger(), "error in areaUpdateTimerCallback: %s", e.what());
            //return;
        }

        // 发布 NavMsg
        publishNavMsg();
    }

    // 状态发布定时器回调函数
    void publishStatusCallback()
    {
        if (bt_.tree) {
            std::string tree_status = bt_.tree_running ? "RUNNING" : "IDLE";
            RCLCPP_DEBUG(this->get_logger(), "行为树状态: %s", tree_status.c_str());
        }
    }

    // 阶段7: 区域管理
    // 初始化区域多边形定义（我将区域范围适当进行了扩大，避免自身坐标结算落到区域外）
    void initializeAreaPolygons()
    {
        // // 红方区域定义（未扩大的）（坐标单位：米，逆时针顺序）
        // // 补给区 (0) - 4顶点
        // area_polygons_[0] = {{0.0f, 8.0f}, {0.0f, 6.0f}, {1.5f, 6.0f}, {1.5f, 8.0f}};

        // // 区域1 (1) - 6顶点
        // area_polygons_[1] = {{0.0f, 6.0f}, {0.0f, 3.0f}, {2.5f, 3.0f},
        //                    {2.5f, 8.0f}, {1.5f, 8.0f}, {1.5f, 6.0f}};

        // // 区域2 (2) - 4顶点
        // area_polygons_[2] = {{0.0f, 3.0f}, {0.0f, 0.0f}, {4.5f, 0.0f}, {4.5f, 3.0f}};

        // // 区域3 (3) - 4顶点
        // area_polygons_[3] = {{4.5f, 2.5f}, {4.5f, 0.0f}, {7.5f, 0.0f}, {7.5f, 2.5f}};

        // // 区域4 (4) - 4顶点 (中心增益点)
        // area_polygons_[4] = {{4.5f, 5.5f}, {4.5f, 2.5f}, {7.5f, 2.5f}, {7.5f, 5.5f}};

        // // 区域5 (5) - 4顶点
        // area_polygons_[5] = {{4.5f, 8.0f}, {4.5f, 5.5f}, {7.5f, 5.5f}, {7.5f, 8.0f}};

        // // 区域6 (6) - 4顶点
        // area_polygons_[6] = {{7.5f, 8.0f}, {7.5f, 5.0f}, {12.0f, 5.0f}, {12.0f, 8.0f}};

        // // 区域7 (7) - 4顶点
        // area_polygons_[7] = {{9.5f, 5.0f}, {9.5f, 0.0f}, {12.0f, 0.0f}, {12.0f, 5.0f}};

        // 红方区域定义（扩大）（坐标单位：米，逆时针顺序）
        // 补给区 (0) - 4顶点
        area_polygons_[0] = {{-1.0f, 9.0f}, {-1.0f, 6.0f}, {1.5f, 6.0f}, {1.5f, 9.0f}};
        // 区域8 (8) - 关于(6,4)的中心对称图形（敌方补给区）
        // 对应变换：(x',y') = (2*6-x, 2*4-y) = (12-x, 8-y)
        area_polygons_[8] = {{13.0f, -1.0f}, {13.0f, 2.0f}, {10.5f, 2.0f}, {10.5f, -1.0f}};

        // 区域1 (1) - 6顶点
        area_polygons_[1] = {{-1.0f, 6.0f}, {-1.0f, 3.0f}, {3.0f, 3.0f},{3.0f, 9.0f}, {1.5f, 9.0f}, {1.5f, 6.0f}};

        // 区域2 (2) - 4顶点
        area_polygons_[2] = {{-1.0f, 3.0f}, {-1.0f, -1.0f}, {4.5f, -1.0f}, {4.5f, 3.0f}};

        // 区域3 (3) - 4顶点
        area_polygons_[3] = {{4.5f, 2.5f}, {4.5f, -1.0f}, {8.0f, -1.0f}, {8.0f, 2.5f}};

        // 区域4 (4) - 4顶点 (中心增益点)
        area_polygons_[4] = {{4.5f, 5.5f}, {4.5f, 2.5f}, {8.0f, 2.5f}, {8.0f, 5.5f}};

        // 区域5 (5) - 4顶点
        area_polygons_[5] = {{4.0f, 9.0f}, {4.0f, 5.5f}, {8.0f, 5.5f}, {8.0f, 9.0f}};

        // 区域6 (6) - 4顶点
        area_polygons_[6] = {{8.0f, 9.0f}, {8.0f, 5.0f}, {13.0f, 5.0f}, {13.0f, 9.0f}};

        // 区域7 (7) - 关于(6,4)的中心对称图形（由区域1镜像得到）
        area_polygons_[7] = {
            {13.0f, 2.0f}, {13.0f, 5.0f}, {9.0f, 5.0f},
            {9.0f, -1.0f}, {10.5f, -1.0f}, {10.5f, 2.0f}
        };
        // 区域8 (8) - 关于(6,4)的中心对称图形（敌方补给区）
        area_polygons_[8] = {{13.0f, -1.0f}, {13.0f, 2.0f}, {10.5f, 2.0f}, {10.5f, -1.0f}};
    }

    // 射线法判断点是否在多边形内
    bool isPointInPolygon(float x, float y, const std::vector<std::pair<float, float>>& polygon)
    {
        bool inside = false;
        size_t n = polygon.size();

        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float x1 = polygon[i].first, y1 = polygon[i].second;
            float x2 = polygon[j].first, y2 = polygon[j].second;

            // 射线法核心判断 (避免除零错误)
            if (((y1 > y) != (y2 > y))) {
                // 避免除零：当y1 == y2时，射线与边平行，不计算交点
                if (y1 != y2) {
                    float intersect_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
                    if (x < intersect_x) {
                        inside = !inside;  // 交点计数翻转
                    }
                }
                // 如果y1 == y2，射线与边平行，不影响inside状态
            }
        }

        return inside;
    }

    // 确定当前区域（坐标单位：米）
    int determineCurrentArea(float field_x, float field_y)
    {
        // 检查补给区 (0)
        auto it = area_polygons_.find(0);
        if (it != area_polygons_.end() &&
            isPointInPolygon(field_x, field_y, it->second)) {
            return 0;
        }

        // 按优先级顺序检查区域 (1-8)
        for (int area_id = 1; area_id <= 8; ++area_id) {
            auto it = area_polygons_.find(area_id);
            if (it != area_polygons_.end() &&
                isPointInPolygon(field_x, field_y, it->second)) {
                return area_id;
            }
        }

        return -1;  // 区域外
    }

    // 更新当前区域（基于机器人位置）
    void updateCurrentArea(float robot_x, float robot_y)
    {
        if (!global_blackboard_) {
            return;
        }

        // 机器人坐标即为场地坐标，无需转换
        float field_x = robot_x;
        float field_y = robot_y;

        // 判断当前区域
        int area_id = determineCurrentArea(field_x, field_y);

        // 更新黑板
        global_blackboard_->set("@current_area", area_id);

        // 获取当前导航目标
        float nav_goal_x = 0.0f, nav_goal_y = 0.0f;
        try {
            auto nav_goal = global_blackboard_->get<geometry_msgs::msg::PoseStamped>("@nav_goal");
            nav_goal_x = nav_goal.pose.position.x;
            nav_goal_y = nav_goal.pose.position.y;
        } catch (...) {

            // 如果获取失败，使用默认值
        }

        // RCLCPP_INFO(this->get_logger(),
        //             "Robot area: team=%s, position=(%.2f,%.2f), area=%d, nav_goal=(%.2f,%.2f)",
        //             current_team_color_ == TeamColor::RED ? "red" : "blue",
        //             robot_x, robot_y, area_id, nav_goal_x, nav_goal_y);

        // 计算敌人距离
        updateEnemyDistances();
    }

    // 更新敌人距离（基于机器人位置）
    void updateEnemyDistances()
    {
        if (!global_blackboard_) {
            return;
        }

        const std::vector<std::string> enemy_types = {"hero", "engineer", "infantry1", "infantry2", "sentry"};
        float min_distance = -1.0f;  // 没有敌人时为-1

        for (const auto& enemy_type : enemy_types) {
            float enemy_distance = -1.0f;  // 默认未检测到

            // 检查是否有任何相机检测到该类型的敌人
            std::string detected_key = "@" + enemy_type + "_detected";
            if (enemy_type == "sentry") {
                detected_key = "@sentry_enemy_detected";
            }

            try {
                auto detected = global_blackboard_->get<int>(detected_key);
                if (detected == 1) {
                    // 使用 updateCameraData 中写入的 @enemy_{type}_distance_{camera_idx} 取最小距离
                    float min_dist_among_cameras = std::numeric_limits<float>::max();
                    int valid_camera_count = 0;

                    for (size_t camera_idx = 0; camera_idx < 4; ++camera_idx) {
                        std::string distance_key = "@enemy_" + enemy_type + "_distance_" + std::to_string(camera_idx);
                        try {
                            float dist = global_blackboard_->get<float>(distance_key);
                            if (dist > 0.0f && dist < min_dist_among_cameras) {
                                min_dist_among_cameras = dist;
                                valid_camera_count++;
                            }
                        } catch (...) {
                            // 该相机未检测到该类型敌人，忽略
                        }
                    }

                    if (valid_camera_count > 0 && min_dist_among_cameras < std::numeric_limits<float>::max()) {
                        enemy_distance = min_dist_among_cameras;

                        // 更新最小距离（所有类型中的最小距离）
                        if (min_distance < 0.0f || enemy_distance < min_distance) {
                            min_distance = enemy_distance;
                        }

                        RCLCPP_DEBUG(this->get_logger(), "敌人%s: 从%d个相机的距离中取最小，距离%.2fm",
                                   enemy_type.c_str(), valid_camera_count, enemy_distance);
                    }
                    // @enemy_{type}_x / _y 已由 updateCameraData 按最新检测更新，此处不再覆盖
                }
            } catch (...) {
                // 忽略异常
            }

            // 更新该类型敌人的距离
            std::string distance_key = "@enemy_" + enemy_type + "_distance";
            global_blackboard_->set(distance_key, enemy_distance);
        }

        // 更新最近敌人距离
        global_blackboard_->set("@enemy_distance_min", min_distance);

        RCLCPP_DEBUG(this->get_logger(),
                    "敌人距离更新: 英雄=%.2fm, 工程=%.2fm, 步兵1=%.2fm, 步兵2=%.2fm, 哨兵=%.2fm, 最近=%.2fm",
                    global_blackboard_->get<float>("@enemy_hero_distance"),
                    global_blackboard_->get<float>("@enemy_engineer_distance"),
                    global_blackboard_->get<float>("@enemy_infantry1_distance"),
                    global_blackboard_->get<float>("@enemy_infantry2_distance"),
                    global_blackboard_->get<float>("@enemy_sentry_distance"),
                    min_distance);
    }

    // 更新英雄所在区域（基于英雄位置）
    void updateHeroArea(float hero_x, float hero_y)
    {
        if (!global_blackboard_) {
            return;
        }

        // 英雄坐标即为场地坐标，无需转换
        float field_x = hero_x;
        float field_y = hero_y;

        // 判断英雄所在区域
        int hero_area_id = determineCurrentArea(field_x, field_y);

        // 更新黑板
        global_blackboard_->set("@hero_area", hero_area_id);

        RCLCPP_DEBUG(this->get_logger(),
                    "Hero area determination: team=%s, hero position=(%.2f,%.2f), area=%d",
                    current_team_color_ == TeamColor::RED ? "红方" : "蓝方",
                    hero_x, hero_y, hero_area_id);
    }

    // 阶段8: 导航关停
    void publishNavMsg()
    {
        if (!comm_.nav_msg || !comm_.status_msg || !global_blackboard_) {
            return;
        }

        rm_interfaces::msg::NavMsg nav_msg;
        rm_interfaces::msg::StatusMsg status_msg;

        int enemy_count = global_blackboard_->get<int>("@enemy_count");
        int sentry_hp = global_blackboard_->get<int>("@sentry_hp");
        int hero_hp = global_blackboard_->get<int>("@hero_hp");
        int infantry_hp = global_blackboard_->get<int>("@infantry_hp");
        int bullets_allowance = global_blackboard_->get<int>("@bullets_allowance");

        // 获取敌方步兵距离
        float enemy_infantry1_distance = global_blackboard_->get<float>("@enemy_infantry1_distance");
        float enemy_infantry2_distance = global_blackboard_->get<float>("@enemy_infantry2_distance");

        auto now = std::chrono::steady_clock::now();

        // 初始化上一次血量
        if (last_sentry_hp_ < 0) {
            last_sentry_hp_ = sentry_hp;
        }
        bool attacked_now = (sentry_hp < last_sentry_hp_);
        if (attacked_now) {
            last_attacked_time_ = now;
        }
        last_sentry_hp_ = sentry_hp;
        bool recently_attacked = (now - last_attacked_time_ < std::chrono::seconds(5));
        
        nav_msg.nav_unable = 1;
        status_msg.sentry_stance = 3;


        if (enemy_count > 0) {
            last_enemy_seen_ = now;
            nav_msg.nav_unable = 0;
            status_msg.sentry_stance = 1;
        } else if (now - last_enemy_seen_ < kNavEnemyKeepDuration) {
            nav_msg.nav_unable = 0;
            status_msg.sentry_stance = 1;
        }

        if (attacked_now && enemy_count <= 0) {
            status_msg.sentry_stance = 2;
        }
        if (recently_attacked) {
            if (enemy_count > 0) {
                status_msg.sentry_stance = 1;
            } else {
                status_msg.sentry_stance = 2;
            }
        }
        if(bullets_allowance < 50){
            status_msg.sentry_stance = 2;
        }

        //(hero_hp < 71 && infantry_hp < 71) ||
        if (sentry_hp < 200 ||
             bullets_allowance < 50 ||
             (enemy_infantry1_distance >= 0.0f && enemy_infantry1_distance < 1.0f) ||
             (enemy_infantry2_distance >= 0.0f && enemy_infantry2_distance < 1.0f)) {
            nav_msg.nav_unable = 1;
        }
        if (sentry_hp < 200) {
            status_msg.sentry_stance = 3;
        }

        // ！！！如果当前区域为2、3、4，跑打
        int current_area = -1;
        try {
            current_area = global_blackboard_->get<int>("@current_area");
        } catch (...) {}

        if (current_area == 2 || current_area == 3 || current_area == 4) {
            nav_msg.nav_unable = 1;
        }

        // 自身与目标距离 ≤ 0.2m 视为到达，禁止导航
        float current_x = global_blackboard_->get<float>("@current_pose_x");
        float current_y = global_blackboard_->get<float>("@current_pose_y");
        float goal_x = 0.0f;
        float goal_y = 0.0f;
        bool has_goal = false;
        {
            std::lock_guard<std::mutex> lock(game_state_mutex_);
            try {
                auto nav_goal = global_blackboard_->get<geometry_msgs::msg::PoseStamped>("@nav_goal");
                goal_x = static_cast<float>(nav_goal.pose.position.x);
                goal_y = static_cast<float>(nav_goal.pose.position.y);
                has_goal = true;
            } catch (const std::exception&) {}
        }
        if (has_goal) {
            float dx = goal_x - current_x;
            float dy = goal_y - current_y;
            float dist = std::sqrt(dx * dx + dy * dy);
            // 开局只判断一次：是否到达 @area4_yellow_patrol（到达后置 1）
            int area4_yellow_patrol_reached = 0;
            try {
                area4_yellow_patrol_reached = global_blackboard_->get<int>("@area4_yellow_patrol_reached");
            } catch (...) {}
            if (area4_yellow_patrol_reached == 0) {
                try {
                    auto patrol_pose = global_blackboard_->get<geometry_msgs::msg::PoseStamped>("@area4_yellow_patrol");
                    float patrol_x = static_cast<float>(patrol_pose.pose.position.x);
                    float patrol_y = static_cast<float>(patrol_pose.pose.position.y);
                    float pdx = patrol_x - current_x;
                    float pdy = patrol_y - current_y;
                    float patrol_dist = std::sqrt(pdx * pdx + pdy * pdy);
                    if (patrol_dist <= 0.2f) {
                        global_blackboard_->set("@area4_yellow_patrol_reached", 1);
                    }
                } catch (...) {
                    // 点位未准备好时忽略
                }
            }
            if (dist <= 0.2f&&enemy_count > 0) {
                last_enemy_seen1_ = now;
                nav_msg.nav_unable = 0;
                status_msg.sentry_stance = 1;
            } else if (now - last_enemy_seen1_ < kNavEnemyKeepDuration) {
            // 在保持期内视为刚漏检，继续允许导航
                nav_msg.nav_unable = 0;
                status_msg.sentry_stance = 1;
            }   
        }
        comm_.nav_msg->publish(nav_msg);
        comm_.status_msg->publish(status_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布 NavMsg: nav_unable=%d (敌人数量:%d, 哨兵血量:%d, 英雄血量:%d, 步兵血量:%d, 子弹限额:%d, 步兵1距离:%.2fm, 步兵2距离:%.2fm)",
                    nav_msg.nav_unable, enemy_count, sentry_hp, hero_hp, infantry_hp, bullets_allowance, enemy_infantry1_distance, enemy_infantry2_distance);
    }

    // 工具
    void logStartupInfo()
    {
        RCLCPP_INFO(this->get_logger(), "RM2026 decision node started");
        RCLCPP_INFO(this->get_logger(), "Tree file: %s", bt_config_.tree_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Tree directory: %s", bt_config_.tree_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "Execution frequency: %.1f Hz", bt_config_.tick_frequency);
    }

private:
    // ===== 分组结构体：TF 组件 =====
    struct TFComponents {
        std::shared_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        rclcpp::TimerBase::SharedPtr timer;
    } tf_;

    // ===== 分组结构体：通信组件 =====
    struct CommunicationComponents {
        rclcpp::Subscription<rm_interfaces::msg::CameraMsg>::SharedPtr camera;
        rclcpp::Subscription<rm_interfaces::msg::ReceiveLLC>::SharedPtr referee;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal;
        rclcpp::Publisher<rm_interfaces::msg::NavMsg>::SharedPtr nav_msg;
        rclcpp::Publisher<rm_interfaces::msg::StatusMsg>::SharedPtr status_msg;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr self_save_status;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_save;
        rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client;
    } comm_;

    // ===== 分组结构体：位置信息 =====
    struct Position2D {
        double x = 0.0;
        double y = 0.0;
    } current_pose_;

    // ===== 分组结构体：行为树组件 =====
    struct BehaviorTreeComponents {
        BT::BehaviorTreeFactory factory;
        std::unique_ptr<BT::Tree> tree;
        std::unique_ptr<BT::Tree> tree_Check;
        std::unique_ptr<BT::Tree> tree_monitor;
        std::unique_ptr<BT::Groot2Publisher> groot2_publisher;
        std::unique_ptr<std::thread> execution_thread;
        std::atomic<bool> stop_execution{false};
        std::atomic<bool> tree_running{false};
    } bt_;

    // ===== 分组结构体：定时器组件 =====
    struct TimerComponents {
        rclcpp::TimerBase::SharedPtr status;
        rclcpp::TimerBase::SharedPtr area_update;
        rclcpp::TimerBase::SharedPtr self_save_status;
        rclcpp::TimerBase::SharedPtr costmap_fetch;
    } timers_;

    // ===== 核心状态（未分组） =====
    TeamColor current_team_color_;
    BT::Blackboard::Ptr global_blackboard_;
    bool referee_data_received_ = false;
    bool auto_restart_ = true;

    /// 导航关停防抖：最近一次检测到敌人的时刻，避免相机几帧漏检导致 nav_unable 抖动
    static constexpr std::chrono::milliseconds kNavEnemyKeepDuration{500};
    std::chrono::steady_clock::time_point last_enemy_seen_{};
    std::chrono::steady_clock::time_point last_enemy_seen1_{};

    // 哨兵血量记忆与被攻击时间
    int last_sentry_hp_ = -1;
    std::chrono::steady_clock::time_point last_attacked_time_{};


    /// 保护 @game_progress / @nav_goal_requested / @nav_goal 的跨线程读写（裁判回调、行为树线程、代价地图回调）
    std::mutex game_state_mutex_;

    // ===== 区域判断相关 =====
    std::map<int, std::vector<std::pair<float, float>>> area_polygons_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<DecisionNode>();
        node->initialize();

        //rclcpp::executors::MultiThreadedExecutor executor;//多线程执行器
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        RCLCPP_INFO(rclcpp::get_logger("rm2026_decision"),
                   "Using single-threaded executor to start \n --------------------------------");

        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rm2026_decision"), "节点启动失败: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
