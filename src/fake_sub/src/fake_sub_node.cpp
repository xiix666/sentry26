#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>
#include <string>
#include <limits>

// RM2026 接口消息
#include <rm_interfaces/msg/camera_msg.hpp>
#include <rm_interfaces/msg/receive_llc.hpp>
#include <rm_interfaces/msg/nav_msg.hpp>

// 标准消息
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// TF2 相关
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FakeSubscriber : public rclcpp::Node
{
public:
    enum class SubscriptionMode {
        ALL = 1,           // 订阅所有话题
        CAMERA_ONLY = 2,   // 只订阅相机数据
        REFEREE_ONLY = 3,  // 只订阅裁判系统数据
        NAVIGATION_ONLY = 4, // 只订阅导航数据
        NAV_MSG_ONLY = 5,  // 只订阅导航控制消息
        TF_ONLY = 6,       // 只订阅TF变换
        CUSTOM = 7         // 自定义选择
    };

    FakeSubscriber()
        : Node("fake_subscriber_node"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_)
    {
        showMenu();
        setupSubscriptions();
        setupTimers();

        // 启动交互线程
        interaction_thread_ = std::thread(&FakeSubscriber::handleUserInput, this);
    }

    ~FakeSubscriber()
    {
        if (interaction_thread_.joinable()) {
            interaction_thread_.join();
        }
    }

private:
    void showMenu()
    {
        std::cout << "\n=======================================" << std::endl;
        std::cout << "  RM2026 数据流监控器" << std::endl;
        std::cout << "=======================================" << std::endl;
        std::cout << "请选择要监控的话题类型:" << std::endl;
        std::cout << "1. 订阅所有话题 (ALL)" << std::endl;
        std::cout << "2. 只订阅相机数据 (CAMERA_ONLY)" << std::endl;
        std::cout << "3. 只订阅裁判系统数据 (REFEREE_ONLY)" << std::endl;
        std::cout << "4. 只订阅导航数据 (NAVIGATION_ONLY)" << std::endl;
        std::cout << "5. 只订阅导航控制消息 (NAV_MSG_ONLY)" << std::endl;
        std::cout << "6. 只订阅TF变换 (TF_ONLY)" << std::endl;
        std::cout << "7. 自定义选择 (CUSTOM)" << std::endl;
        std::cout << "=======================================" << std::endl;
        std::cout << "请输入选择 (1-7): ";

        int choice;
        while (true) {
            std::cin >> choice;
            if (std::cin.fail() || choice < 1 || choice > 7) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "输入无效，请输入1-7之间的数字: ";
            } else {
                break;
            }
        }

        subscription_mode_ = static_cast<SubscriptionMode>(choice);

        if (subscription_mode_ == SubscriptionMode::CUSTOM) {
            showCustomMenu();
        }

        std::cout << "\n开始监控数据流..." << std::endl;
        std::cout << "按 'q' + Enter 退出" << std::endl;
        std::cout << "=======================================\n" << std::endl;
    }

    void showCustomMenu()
    {
        std::cout << "\n自定义选择 - 选择要订阅的话题:" << std::endl;
        std::cout << "1. 相机数据 (/gimbal_camera_topic) [y/n]: ";
        char choice;
        std::cin >> choice;
        subscribe_camera_ = (choice == 'y' || choice == 'Y');

        std::cout << "2. 裁判系统数据 (/receiveLLC_pack) [y/n]: ";
        std::cin >> choice;
        subscribe_referee_ = (choice == 'y' || choice == 'Y');

        std::cout << "3. 导航目标 (/goal_pose) [y/n]: ";
        std::cin >> choice;
        subscribe_nav_goal_ = (choice == 'y' || choice == 'Y');

        std::cout << "4. 导航状态 (navigation/status) [y/n]: ";
        std::cin >> choice;
        subscribe_nav_status_ = (choice == 'y' || choice == 'Y');

        std::cout << "5. 导航控制消息 (/nav_pack) [y/n]: ";
        std::cin >> choice;
        subscribe_nav_msg_ = (choice == 'y' || choice == 'Y');

        std::cout << "6. TF变换 (map->base_link) [y/n]: ";
        std::cin >> choice;
        subscribe_tf_ = (choice == 'y' || choice == 'Y');
    }

    void handleUserInput()
    {
        while (rclcpp::ok()) {
            std::string input;
            std::getline(std::cin, input);

            if (input == "q" || input == "Q") {
                std::cout << "\n收到退出信号，正在关闭..." << std::endl;
                rclcpp::shutdown();
                break;
            }
        }
    }
    void setupSubscriptions()
    {
        std::cout << "\n正在初始化订阅器..." << std::endl;

        // 根据选择创建订阅器
        switch (subscription_mode_) {
            case SubscriptionMode::ALL:
                subscribe_camera_ = true;
                subscribe_referee_ = true;
                subscribe_nav_goal_ = true;
                subscribe_nav_status_ = true;
                subscribe_nav_msg_ = true;
                subscribe_tf_ = true;
                break;
            case SubscriptionMode::CAMERA_ONLY:
                subscribe_camera_ = true;
                break;
            case SubscriptionMode::REFEREE_ONLY:
                subscribe_referee_ = true;
                break;
            case SubscriptionMode::NAVIGATION_ONLY:
                subscribe_nav_goal_ = true;
                subscribe_nav_status_ = true;
                break;
            case SubscriptionMode::NAV_MSG_ONLY:
                subscribe_nav_msg_ = true;
                break;
            case SubscriptionMode::TF_ONLY:
                subscribe_tf_ = true;
                break;
            case SubscriptionMode::CUSTOM:
                // 已经在showCustomMenu中设置
                break;
        }

        // 创建订阅器
        if (subscribe_camera_) {
            camera_subscriber_ = this->create_subscription<rm_interfaces::msg::CameraMsg>(
                "/gimbal_camera_topic", 10,
                [this](const rm_interfaces::msg::CameraMsg::SharedPtr msg) {
                    handleCameraData(msg);
                });
            std::cout << "✓ 已订阅: /camera_topic" << std::endl;
        }

        if (subscribe_referee_) {
            referee_subscriber_ = this->create_subscription<rm_interfaces::msg::ReceiveLLC>(
                "/receiveLLC_pack", 10,
                [this](const rm_interfaces::msg::ReceiveLLC::SharedPtr msg) {
                    handleRefereeData(msg);
                });
            std::cout << "✓ 已订阅: /receiveLLC_pack" << std::endl;
        }

        if (subscribe_nav_goal_) {
            navigation_goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    handleNavigationGoal(msg);
                });
            std::cout << "✓ 已订阅: /goal_pose" << std::endl;
        }

        if (subscribe_nav_status_) {
            navigation_status_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
                "navigation/status", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    handleNavigationStatus(msg);
                });
            std::cout << "✓ 已订阅: navigation/status" << std::endl;
        }

        if (subscribe_nav_msg_) {
            nav_msg_subscriber_ = this->create_subscription<rm_interfaces::msg::NavMsg>(
                "/nav_pack", 10,
                [this](const rm_interfaces::msg::NavMsg::SharedPtr msg) {
                    handleNavMsg(msg);
                });
            std::cout << "✓ 已订阅: /nav_pack" << std::endl;
        }

        if (subscribe_tf_) {
            // TF监听器已经在构造函数中初始化
            std::cout << "✓ 已订阅: TF变换 (map->base_link)" << std::endl;
        }

        std::cout << "订阅器初始化完成！\n" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Fake Subscriber 订阅器初始化完成");
    }

    void setupTimers()
    {
        if (subscribe_tf_) {
            // TF监听定时器 (10Hz)
            tf_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),  // 10Hz
                [this]() { checkTFTransform(); });

            std::cout << "✓ TF监听定时器已启动 (10Hz)" << std::endl;
            RCLCPP_INFO(this->get_logger(), "TF监听定时器初始化完成 (10Hz)");
        }
    }

    void handleCameraData(const rm_interfaces::msg::CameraMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "=== 收到相机数据 ===");
        RCLCPP_INFO(this->get_logger(), "时间戳: %d.%d",
                   msg->header.stamp.sec, msg->header.stamp.nanosec);

        // 显示每个相机的检测结果
        const std::vector<std::string> vehicle_types = {"hero", "engineer", "infantry1", "infantry2", "sentry"};
        const std::vector<const std::array<int32_t, 5>*> id_arrays = {
            &msg->id_1, &msg->id_2, &msg->id_3, &msg->id_4
        };
        const std::vector<const std::array<float, 5>*> pose_x_arrays = {
            &msg->enemy_pose_1_x, &msg->enemy_pose_2_x, &msg->enemy_pose_3_x, &msg->enemy_pose_4_x
        };
        const std::vector<const std::array<float, 5>*> pose_y_arrays = {
            &msg->enemy_pose_1_y, &msg->enemy_pose_2_y, &msg->enemy_pose_3_y, &msg->enemy_pose_4_y
        };

        for (int camera_idx = 1; camera_idx <= 4; ++camera_idx) {
            const auto& id_array = *id_arrays[camera_idx - 1];
            const auto& pose_x_array = *pose_x_arrays[camera_idx - 1];
            const auto& pose_y_array = *pose_y_arrays[camera_idx - 1];

            std::stringstream camera_info;
            camera_info << "相机" << camera_idx << ": ";

            bool has_detections = false;
            for (size_t vehicle_idx = 0; vehicle_idx < vehicle_types.size(); ++vehicle_idx) {
                if (id_array[vehicle_idx] != 0) {
                    if (has_detections) camera_info << ", ";
                    camera_info << vehicle_types[vehicle_idx] << "("
                               << pose_x_array[vehicle_idx] << "," << pose_y_array[vehicle_idx] << ")";
                    has_detections = true;
                }
            }

            if (has_detections) {
                RCLCPP_INFO(this->get_logger(), "%s", camera_info.str().c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "相机%d: 无敌人检测", camera_idx);
            }
        }

        // 显示速度信息
        if (!msg->speed_x.empty() && !msg->speed_y.empty()) {
            RCLCPP_INFO(this->get_logger(), "速度信息 - X: [%.2f, %.2f, %.2f, %.2f, %.2f]",
                       msg->speed_x[0], msg->speed_x[1], msg->speed_x[2], msg->speed_x[3], msg->speed_x[4]);
            RCLCPP_INFO(this->get_logger(), "速度信息 - Y: [%.2f, %.2f, %.2f, %.2f, %.2f]",
                       msg->speed_y[0], msg->speed_y[1], msg->speed_y[2], msg->speed_y[3], msg->speed_y[4]);
        }
    }

    void handleRefereeData(const rm_interfaces::msg::ReceiveLLC::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "=== 收到裁判系统数据 ===");
        RCLCPP_INFO(this->get_logger(), "Game Progress: %d, Stage Remaining Time: %d seconds",
                   msg->game_progress, msg->stage_remain_time);
        RCLCPP_INFO(this->get_logger(), "Team: %s (Red=1, Blue=0)",
                   msg->red_blue == 0 ? "Blue" : "Red");

        RCLCPP_INFO(this->get_logger(), "Self Health - Hero: %d, Infantry: %d, Sentry: %d",
                   msg->self_hero_hp, msg->self_infantry_hp, msg->self_sentry_hp);
        RCLCPP_INFO(this->get_logger(), "Ammo Limit: %d",
                   msg->bullets_allowance);

        // 显示哨兵与队友位置
        RCLCPP_INFO(this->get_logger(), "Sentry: (%.2f, %.2f), Hero: (%.2f, %.2f), Infantry: (%.2f, %.2f)",
                   msg->self_pose_x, msg->self_pose_y,
                   msg->hero_pose_x, msg->hero_pose_y,
                   msg->infantry_pose_x, msg->infantry_pose_y);

        // 显示联盟赛信息
        RCLCPP_INFO(this->get_logger(), "Alliance Game - Center Gain Point Status: %d, Remaining Gold: %d",
                   msg->center_status, msg->remain_gold);
    }

    void handleNavigationGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "=== 收到导航目标 ===");
        RCLCPP_INFO(this->get_logger(), "Target Position: (%.2f, %.2f, %.2f)",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Coordinate System: %s", msg->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "Timestamp: %d.%d",
                   msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    void handleNavigationStatus(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "=== 收到导航状态 ===");
        RCLCPP_INFO(this->get_logger(), "Navigation Completed: %s", msg->data ? "Yes" : "No");
    }

    void handleNavMsg(const rm_interfaces::msg::NavMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "=== 收到导航控制消息 ===");
        RCLCPP_INFO(this->get_logger(), "导航禁用状态 (nav_unable): %d", msg->nav_unable);
    }

    void checkTFTransform()
    {
        try {
            // 查询 map 到 base_link 的变换
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_->lookupTransform("map", "base_link", tf2::TimePoint());

            RCLCPP_INFO(this->get_logger(), "=== TF变换: map -> base_link ===");
            RCLCPP_INFO(this->get_logger(), "位置: (%.3f, %.3f, %.3f)",
                       transformStamped.transform.translation.x,
                       transformStamped.transform.translation.y,
                       transformStamped.transform.translation.z);

            // 将四元数转换为欧拉角显示
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), "姿态 (欧拉角): 横滚=%.3f°, 俯仰=%.3f°, 偏航=%.3f°",
                       roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

            RCLCPP_INFO(this->get_logger(), "时间戳: %d.%d",
                       transformStamped.header.stamp.sec,
                       transformStamped.header.stamp.nanosec);

        } catch (const tf2::TransformException &ex) {
            // 只在第一次失败时输出警告，避免日志过多
            static bool first_tf_error = true;
            if (first_tf_error) {
                RCLCPP_WARN(this->get_logger(), "TF查询失败 (这很正常，如果fake_publisher还没启动): %s", ex.what());
                first_tf_error = false;
            }
        }
    }

    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // 订阅器
    rclcpp::Subscription<rm_interfaces::msg::CameraMsg>::SharedPtr camera_subscriber_;
    rclcpp::Subscription<rm_interfaces::msg::ReceiveLLC>::SharedPtr referee_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_status_subscriber_;
    rclcpp::Subscription<rm_interfaces::msg::NavMsg>::SharedPtr nav_msg_subscriber_;

    // 订阅配置
    SubscriptionMode subscription_mode_;
    bool subscribe_camera_ = false;
    bool subscribe_referee_ = false;
    bool subscribe_nav_goal_ = false;
    bool subscribe_nav_status_ = false;
    bool subscribe_nav_msg_ = false;
    bool subscribe_tf_ = false;

    // 交互线程
    std::thread interaction_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<FakeSubscriber>();

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        RCLCPP_INFO(rclcpp::get_logger("fake_sub"),
                   "Fake Subscriber 使用单线程执行器启动");

        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fake_sub"), "节点启动失败: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}