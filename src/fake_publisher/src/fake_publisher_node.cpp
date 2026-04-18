#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>

// RM2026 接口消息
#include <rm_interfaces/msg/camera_msg.hpp>
#include <rm_interfaces/msg/receive_llc.hpp>

// 标准消息
#include <geometry_msgs/msg/pose_stamped.hpp>

// TF2 相关
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>

// YAML配置
#include <yaml-cpp/yaml.h>

class FakePublisher : public rclcpp::Node
{
public:
    FakePublisher()
        : Node("fake_publisher_node"),
          robot_x_(1.5),  // 机器人初始X位置
          robot_y_(4.0),  // 机器人初始Y位置
          robot_yaw_(0.0), // 机器人初始朝向
        //   camera_publish_rate_(10.0),  // 10Hz
          referee_publish_rate_(10.0), // 10Hz
          // tf_publish_rate_(50.0),      // 50Hz TF广播
          navigation_response_delay_(1.0), // 1秒延迟响应
          running_(true)
    {
        declareParameters();
        loadConfig();  // 加载YAML配置
        setupPublishers();
        setupSubscriptions();
        setupTimers();

        RCLCPP_INFO(this->get_logger(), "Fake Publisher 节点已启动");
        // RCLCPP_INFO(this->get_logger(), "相机数据发布频率: %.1f Hz", camera_publish_rate_);
        RCLCPP_INFO(this->get_logger(), "裁判系统数据发布频率: %.1f Hz", referee_publish_rate_);
    }

    ~FakePublisher()
    {
        running_ = false;
        if (navigation_response_thread_ && navigation_response_thread_->joinable())
        {
            navigation_response_thread_->join();
        }
    }

private:
    void declareParameters()
    {
        // 相机数据参数
        // this->declare_parameter<double>("camera_publish_rate", 10.0);

        // 裁判系统参数
        this->declare_parameter<double>("referee_publish_rate", 10.0);

        // TF广播参数
        // this->declare_parameter<double>("tf_publish_rate", 50.0);
        this->declare_parameter<double>("robot_initial_x", 0.75);//修改机器人位置
        this->declare_parameter<double>("robot_initial_y", 7.0);
        this->declare_parameter<double>("robot_initial_yaw", 0.0);

        // 导航响应参数
        this->declare_parameter<double>("navigation_response_delay", 1.0);
    }

    void loadConfig()
    {
        try {
            // 获取包的share目录路径
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("fake_publisher");
            std::string config_file = package_share_directory + "/config/fake_publisher_config.yaml";

            // 加载YAML文件
            YAML::Node config = YAML::LoadFile(config_file);

            // 加载裁判系统配置
            auto referee = config["referee"];
            referee_config_.game_progress = referee["game_progress"].as<int>();
            referee_config_.stage_remain_time = referee["stage_remain_time"].as<int>();
            referee_config_.red_blue = referee["red_blue"].as<int>();
            referee_config_.self_hero_hp = referee["self_hero_hp"].as<int>();
            referee_config_.self_infantry_hp = referee["self_infantry_hp"].as<int>();
            referee_config_.self_sentry_hp = referee["self_sentry_hp"].as<int>();
            referee_config_.bullets_allowance = referee["bullets_allowance"].as<int>();
            referee_config_.center_status = referee["center_status"].as<int>();
            referee_config_.remain_gold = referee["remain_gold"].as<int>();
            referee_config_.self_pose_x = referee["self_pose_x"].as<float>();
            referee_config_.self_pose_y = referee["self_pose_y"].as<float>();
            referee_config_.hero_pose_x = referee["hero_pose_x"].as<float>();
            referee_config_.hero_pose_y = referee["hero_pose_y"].as<float>();
            referee_config_.infantry_pose_x = referee["infantry_pose_x"].as<float>();
            referee_config_.infantry_pose_y = referee["infantry_pose_y"].as<float>();

            // 加载相机配置
            // auto cameras = config["cameras"];
            // std::vector<std::string> enemy_types = {"hero", "engineer", "infantry1", "infantry2", "sentry"};

            // for (int camera_idx = 0; camera_idx < 4; ++camera_idx) {
            //     std::string camera_key = "camera_" + std::to_string(camera_idx + 1);
            //     auto camera = cameras[camera_key];

            //     for (int enemy_idx = 0; enemy_idx < 5; ++enemy_idx) {
            //         auto enemy = camera[enemy_types[enemy_idx]];
            //         camera_config_.enemies[camera_idx][enemy_idx].id = enemy["id"].as<int32_t>();
            //         camera_config_.enemies[camera_idx][enemy_idx].x = enemy["x"].as<float>();
            //         camera_config_.enemies[camera_idx][enemy_idx].y = enemy["y"].as<float>();
            //     }
            // }

            // // 加载速度配置
            // auto speeds = config["enemy_speeds"];
            // speed_config_.hero_x = speeds["hero"]["x"].as<float>();
            // speed_config_.hero_y = speeds["hero"]["y"].as<float>();
            // speed_config_.engineer_x = speeds["engineer"]["x"].as<float>();
            // speed_config_.engineer_y = speeds["engineer"]["y"].as<float>();
            // speed_config_.infantry1_x = speeds["infantry1"]["x"].as<float>();
            // speed_config_.infantry1_y = speeds["infantry1"]["y"].as<float>();
            // speed_config_.infantry2_x = speeds["infantry2"]["x"].as<float>();
            // speed_config_.infantry2_y = speeds["infantry2"]["y"].as<float>();
            // speed_config_.sentry_x = speeds["sentry"]["x"].as<float>();
            // speed_config_.sentry_y = speeds["sentry"]["y"].as<float>();

            RCLCPP_INFO(this->get_logger(), "成功加载YAML配置文件: %s", config_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "加载YAML配置文件失败: %s", e.what());
            // 使用默认配置
            loadDefaultConfig();
        }
    }

    void loadDefaultConfig()
    {
        // 裁判系统默认配置
        referee_config_.game_progress = 4;
        referee_config_.stage_remain_time = 300;
        referee_config_.red_blue = 1;
        referee_config_.self_hero_hp = 210;
        referee_config_.self_infantry_hp = 210;
        referee_config_.self_sentry_hp = 400;
        referee_config_.bullets_allowance = 300;
        referee_config_.center_status = 2;
        referee_config_.remain_gold = 0;
        referee_config_.self_pose_x = 1.5f;
        referee_config_.self_pose_y = 4.0f;
        referee_config_.hero_pose_x = 5.0f;
        referee_config_.hero_pose_y = 1.5f;
        referee_config_.infantry_pose_x = 5.0f;
        referee_config_.infantry_pose_y = 1.5f;

        // 相机默认配置
        // 相机1
        camera_config_.enemies[0][0] = {1, 0.75f, 6.0f};   // hero
        camera_config_.enemies[0][1] = {2, 7.5f, 5.5f};    // engineer
        camera_config_.enemies[0][2] = {3, 7.0f, 5.0f};    // infantry1
        camera_config_.enemies[0][3] = {4, 6.5f, 4.5f};    // infantry2
        camera_config_.enemies[0][4] = {5, 6.0f, 4.0f};    // sentry

        // 相机2
        camera_config_.enemies[1][0] = {6, 2.0f, 7.0f};
        camera_config_.enemies[1][1] = {7, 1.5f, 6.5f};
        camera_config_.enemies[1][2] = {8, 1.0f, 6.0f};
        camera_config_.enemies[1][3] = {9, 3.0f, 6.0f};
        camera_config_.enemies[1][4] = {10, 2.5f, 5.5f};

        // 相机3
        camera_config_.enemies[2][0] = {11, 9.0f, 5.0f};
        camera_config_.enemies[2][1] = {12, 9.5f, 4.5f};
        camera_config_.enemies[2][2] = {13, 10.0f, 4.0f};
        camera_config_.enemies[2][3] = {14, 10.5f, 3.5f};
        camera_config_.enemies[2][4] = {15, 11.0f, 3.0f};

        // 相机4
        camera_config_.enemies[3][0] = {16, 6.0f, 0.5f};
        camera_config_.enemies[3][1] = {17, 5.5f, 0.8f};
        camera_config_.enemies[3][2] = {18, 6.5f, 0.8f};
        camera_config_.enemies[3][3] = {19, 7.0f, 0.5f};
        camera_config_.enemies[3][4] = {20, 5.0f, 1.0f};

        // 速度默认配置
        speed_config_.hero_x = -1.2f;
        speed_config_.hero_y = 0.0f;
        speed_config_.engineer_x = 0.5f;
        speed_config_.engineer_y = 0.0f;
        speed_config_.infantry1_x = 0.8f;
        speed_config_.infantry1_y = 0.0f;
        speed_config_.infantry2_x = -0.3f;
        speed_config_.infantry2_y = 0.0f;
        speed_config_.sentry_x = 1.5f;
        speed_config_.sentry_y = -0.8f;

        RCLCPP_WARN(this->get_logger(), "使用默认配置");
    }

    // 发布裁判系统数据
    void publishRefereeData()
    {
        auto msg = std::make_shared<rm_interfaces::msg::ReceiveLLC>();

        // 使用配置文件中的数据
        msg->game_progress = referee_config_.game_progress;
        msg->stage_remain_time = referee_config_.stage_remain_time;
        msg->red_blue = referee_config_.red_blue;

        msg->self_hero_hp = referee_config_.self_hero_hp;
        msg->self_infantry_hp = referee_config_.self_infantry_hp;
        msg->self_sentry_hp = referee_config_.self_sentry_hp;

        msg->bullets_allowance = referee_config_.bullets_allowance;
        // msg->chassis_power = referee_config_.chassis_power;  // 暂时注释

        msg->center_status = referee_config_.center_status;
        msg->remain_gold = referee_config_.remain_gold;

        msg->self_pose_x = referee_config_.self_pose_x;
        msg->self_pose_y = referee_config_.self_pose_y;
        msg->hero_pose_x = referee_config_.hero_pose_x;
        msg->hero_pose_y = referee_config_.hero_pose_y;
        msg->infantry_pose_x = referee_config_.infantry_pose_x;
        msg->infantry_pose_y = referee_config_.infantry_pose_y;

        referee_publisher_->publish(*msg);

        static int count = 0;
        if (++count % static_cast<int>(referee_publish_rate_) == 0) {
            RCLCPP_INFO(this->get_logger(), "发布裁判系统数据: 队伍=%s, 血量=%d/%d/%d, 弹药=%d",
                       msg->red_blue == 0 ? "蓝方" : "红方",
                       msg->self_hero_hp, msg->self_infantry_hp, msg->self_sentry_hp,
                       msg->bullets_allowance);
        }
    }
    void setupPublishers()
    {
        // 相机数据发布器
        // camera_publisher_ = this->create_publisher<rm_interfaces::msg::CameraMsg>(
        //     "/camera_topic", 10);

        // 裁判系统数据发布器
        referee_publisher_ = this->create_publisher<rm_interfaces::msg::ReceiveLLC>(
            "/receiveLLC_pack", 10);

        // TF广播器
        // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "发布器初始化完成");
    }

    void setupSubscriptions()
    {
        // 订阅导航目标点
        navigation_goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                handleNavigationGoal(msg);
            });

        RCLCPP_INFO(this->get_logger(), "订阅器初始化完成");
    }

    void setupTimers()
    {
        // 相机数据定时器
        // camera_publish_rate_ = this->get_parameter("camera_publish_rate").as_double();
        // auto camera_period = std::chrono::duration<double>(1.0 / camera_publish_rate_);
        // camera_timer_ = this->create_wall_timer(
        //     std::chrono::duration_cast<std::chrono::milliseconds>(camera_period),
        //     [this]() { publishCameraData(); });

        // 裁判系统数据定时器
        referee_publish_rate_ = this->get_parameter("referee_publish_rate").as_double();
        auto referee_period = std::chrono::duration<double>(1.0 / referee_publish_rate_);
        referee_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(referee_period),
            [this]() { publishRefereeData(); });

        // TF广播定时器
        // tf_publish_rate_ = this->get_parameter("tf_publish_rate").as_double();
        // auto tf_period = std::chrono::duration<double>(1.0 / tf_publish_rate_);
        // tf_timer_ = this->create_wall_timer(
        //     std::chrono::duration_cast<std::chrono::milliseconds>(tf_period),
        //     [this]() { publishTFTransform(); });

        // 初始化机器人位置
        robot_x_ = this->get_parameter("robot_initial_x").as_double();
        robot_y_ = this->get_parameter("robot_initial_y").as_double();
        robot_yaw_ = this->get_parameter("robot_initial_yaw").as_double();

        RCLCPP_INFO(this->get_logger(), "定时器初始化完成");
        RCLCPP_INFO(this->get_logger(), "机器人初始位置: (%.2f, %.2f, %.2f)",
                   robot_x_, robot_y_, robot_yaw_);
    }

    // 发布相机数据
    // void publishCameraData()
    // {
    //     auto msg = std::make_shared<rm_interfaces::msg::CameraMsg>();

    //     // 设置消息头
    //     msg->header.stamp = this->get_clock()->now();
    //     msg->header.frame_id = "camera_frame";

    //     // 初始化所有ID为0 (未检测到) - 现在通过配置文件设置

    //     // 初始化位置
    //     for (size_t i = 0; i < 5; ++i) {
    //         msg->enemy_pose_1_x[i] = 0.0;
    //         msg->enemy_pose_1_y[i] = 0.0;
    //         msg->enemy_pose_2_x[i] = 0.0;
    //         msg->enemy_pose_2_y[i] = 0.0;
    //         msg->enemy_pose_3_x[i] = 0.0;
    //         msg->enemy_pose_3_y[i] = 0.0;
    //         msg->enemy_pose_4_x[i] = 0.0;
    //         msg->enemy_pose_4_y[i] = 0.0;
    //     }

    //     // 初始化速度
    //     msg->speed_x = {0.0, 0.0, 0.0, 0.0, 0.0};
    //     msg->speed_y = {0.0, 0.0, 0.0, 0.0, 0.0};

    //     // 使用配置文件设置每个相机的敌人信息
    //     // 每个相机都设置所有5种类型的敌人（英雄、工程、步兵1、步兵2、哨兵）

    //     // 设置相机位置和ID信息
    //     for (int camera_idx = 0; camera_idx < 4; ++camera_idx) {
    //         float* x_arrays[4] = {msg->enemy_pose_1_x.data(), msg->enemy_pose_2_x.data(),
    //                              msg->enemy_pose_3_x.data(), msg->enemy_pose_4_x.data()};
    //         float* y_arrays[4] = {msg->enemy_pose_1_y.data(), msg->enemy_pose_2_y.data(),
    //                              msg->enemy_pose_3_y.data(), msg->enemy_pose_4_y.data()};
    //         int32_t* id_arrays[4] = {msg->id_1.data(), msg->id_2.data(),
    //                                 msg->id_3.data(), msg->id_4.data()};

    //         for (int enemy_idx = 0; enemy_idx < 5; ++enemy_idx) {
    //             id_arrays[camera_idx][enemy_idx] = camera_config_.enemies[camera_idx][enemy_idx].id;
    //             x_arrays[camera_idx][enemy_idx] = camera_config_.enemies[camera_idx][enemy_idx].x;
    //             y_arrays[camera_idx][enemy_idx] = camera_config_.enemies[camera_idx][enemy_idx].y;
    //         }
    //     }

    //     // 设置不同类型敌人的速度
    //     msg->speed_x[0] = speed_config_.hero_x;
    //     msg->speed_y[0] = speed_config_.hero_y;

    //     msg->speed_x[1] = speed_config_.engineer_x;
    //     msg->speed_y[1] = speed_config_.engineer_y;

    //     msg->speed_x[2] = speed_config_.infantry1_x;
    //     msg->speed_y[2] = speed_config_.infantry1_y;

    //     msg->speed_x[3] = speed_config_.infantry2_x;
    //     msg->speed_y[3] = speed_config_.infantry2_y;

    //     msg->speed_x[4] = speed_config_.sentry_x;
    //     msg->speed_y[4] = speed_config_.sentry_y;

    //     // camera_publisher_->publish(*msg);


    //     // 构建日志输出，显示所有检测到的敌人
    //     static int count = 0;
    //     if (++count % static_cast<int>(camera_publish_rate_) == 0) {
    //         const std::vector<std::string> vehicle_types = {"hero", "engineer", "infantry1", "infantry2", "sentry"};
    //         const std::vector<std::array<int32_t, 5>> id_arrays = {msg->id_1, msg->id_2, msg->id_3, msg->id_4};

    //         std::stringstream log_msg;
    //         log_msg << "相机数据发布:";

    //         bool has_detections = false;
    //         for (int camera_idx = 1; camera_idx <= 4; ++camera_idx) {
    //             const auto& id_array = id_arrays[camera_idx - 1];
    //             bool camera_has_detections = false;

    //             for (size_t vehicle_idx = 0; vehicle_idx < vehicle_types.size(); ++vehicle_idx) {
    //                 if (id_array[vehicle_idx] != 0) {
    //                     if (!has_detections) {
    //                         log_msg << "\n";
    //                         has_detections = true;
    //                     }
    //                     if (!camera_has_detections) {
    //                         log_msg << "  相机" << camera_idx << ": ";
    //                         camera_has_detections = true;
    //                     } else {
    //                         log_msg << ", ";
    //                     }
    //                     log_msg << vehicle_types[vehicle_idx] << "(ID:" << id_array[vehicle_idx] << ")";
    //                 }
    //             }
    //         }

    //         if (!has_detections) {
    //             log_msg << " 无敌人检测";
    //         }

    //         RCLCPP_INFO(this->get_logger(), "%s", log_msg.str().c_str());
    //     }
    // }

    // void publishTFTransform()
    // {
    //     geometry_msgs::msg::TransformStamped transformStamped;

    //     // 设置变换消息
    //     transformStamped.header.stamp = this->get_clock()->now();
    //     transformStamped.header.frame_id = "map";        // 父坐标系
    //     transformStamped.child_frame_id = "base_link";   // 子坐标系

    //     // 设置位置
    //     transformStamped.transform.translation.x = robot_x_;
    //     transformStamped.transform.translation.y = robot_y_;
    //     transformStamped.transform.translation.z = 0.0;

    //     // 设置旋转 (四元数)
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, robot_yaw_);  // 绕Z轴旋转
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();

    //     // 广播变换
    //     tf_broadcaster_->sendTransform(transformStamped);

    //     // 定期输出位置信息
    //     static int count = 0;
    //     if (++count % static_cast<int>(tf_publish_rate_) == 0) {
    //         RCLCPP_INFO(this->get_logger(), "TF广播: map -> base_link (%.2f, %.2f, %.2f)",
    //                    robot_x_, robot_y_, robot_yaw_);
    //     }
    // }

    void handleNavigationGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到导航目标: (%.2f, %.2f)",
                   msg->pose.position.x, msg->pose.position.y);

        // 启动异步响应线程
        if (navigation_response_thread_ && navigation_response_thread_->joinable()) {
            navigation_response_thread_->join();
        }

        navigation_response_thread_ = std::make_unique<std::thread>(
            [this]() {
                // 延迟响应 - 直接设置延迟时间
                double delay = 1.0;  // 1秒延迟
                std::this_thread::sleep_for(std::chrono::duration<double>(delay));

                if (running_) {
                    // 导航延迟结束（决策侧按距离判断到达，此处仅做延迟）
                }
            });
    }

    // 发布器
    // rclcpp::Publisher<rm_interfaces::msg::CameraMsg>::SharedPtr camera_publisher_;
    rclcpp::Publisher<rm_interfaces::msg::ReceiveLLC>::SharedPtr referee_publisher_;
    // TF广播器
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 订阅器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_subscriber_;

    // 定时器
    // rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::TimerBase::SharedPtr referee_timer_;
    // rclcpp::TimerBase::SharedPtr tf_timer_;

    // 参数
    // double camera_publish_rate_;
    double referee_publish_rate_;
    // double tf_publish_rate_;
    double navigation_response_delay_;

    // 机器人状态
    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    // 多线程
    std::atomic<bool> running_;
    std::unique_ptr<std::thread> navigation_response_thread_;

    // 配置数据
    struct RefereeConfig {
        int game_progress;
        int stage_remain_time;
        int red_blue;
        int self_hero_hp;
        int self_infantry_hp;
        int self_sentry_hp;
        int bullets_allowance;
        int center_status;
        int remain_gold;
        float self_pose_x, self_pose_y;
        float hero_pose_x, hero_pose_y;
        float infantry_pose_x, infantry_pose_y;
    };

    struct CameraConfig {
        struct EnemyInfo {
            int32_t id;
            float x, y;
        };
        EnemyInfo enemies[4][5];  // 4个相机，每个相机5种敌人类型
    };

    struct SpeedConfig {
        float hero_x, hero_y;
        float engineer_x, engineer_y;
        float infantry1_x, infantry1_y;
        float infantry2_x, infantry2_y;
        float sentry_x, sentry_y;
    };

    RefereeConfig referee_config_;
    CameraConfig camera_config_;
    SpeedConfig speed_config_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<FakePublisher>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        RCLCPP_INFO(rclcpp::get_logger("fake_publisher"),
                   "Fake Publisher 使用多线程执行器启动");

        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fake_publisher"), "节点启动失败: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}