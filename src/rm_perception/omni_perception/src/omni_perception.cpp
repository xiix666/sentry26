#include <rclcpp/rclcpp.hpp>

#include <rm_interfaces/msg/angle_msg.hpp>
#include <rm_interfaces/msg/camera_msg.hpp>
#include <rm_interfaces/msg/send_to_lidar_msg.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/point32.hpp"
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <array>
#include <mutex>

class OmniPerception : public rclcpp::Node
{
public:
    OmniPerception() : Node("OmniPerception")
    {
        this->declare_parameter<bool>("red_blue", 0);
        this->declare_parameter<float>("transform_red_x", 0.0f);
        this->declare_parameter<float>("transform_red_y", 0.0f);
        this->declare_parameter<float>("transform_blue_x", 0.0f);
        this->declare_parameter<float>("transform_blue_y", 0.0f);
        this->declare_parameter<std::string>("aim_odom_frame", "odom");
        this->declare_parameter<std::string>("gimbal_frame", "big_gimbal_link");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<double>("aim_delay", 0.4);
        this->declare_parameter<double>("cmd_vel_timeout", 0.2);
        this->declare_parameter<bool>("enable_motion_compensation", true);
        this->declare_parameter<std::string>("map_frame", "map");
        this->get_parameter("map_frame", map_frame_);
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        aim_delay_ = this->get_parameter("aim_delay").as_double();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        enable_motion_compensation_ = this->get_parameter("enable_motion_compensation").as_bool();

        red_blue = this->get_parameter("red_blue").as_bool();
        transform_red_x_ = this->get_parameter("transform_red_x").as_double();
        transform_red_y_ = this->get_parameter("transform_red_y").as_double();
        transform_blue_x_ = this->get_parameter("transform_blue_x").as_double();
        transform_blue_y_ = this->get_parameter("transform_blue_y").as_double();
        aim_odom_frame_ = this->get_parameter("aim_odom_frame").as_string();
        gimbal_frame_ = this->get_parameter("gimbal_frame").as_string();

        camera_sub_ = this->create_subscription<rm_interfaces::msg::CameraMsg>(
        "/camera_topic", 10, [this](const rm_interfaces::msg::CameraMsg::SharedPtr msg) {
            cameraCallback(msg);
        });
        gimbal_camera_pub_ = this->create_publisher<rm_interfaces::msg::CameraMsg>(
            "/gimbal_camera_topic", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        angle_pub_ = this->create_publisher<geometry_msgs::msg::Point32>("/angle_pack", rclcpp::SensorDataQoS());

        send_to_lidar_pub_ = this->create_publisher<rm_interfaces::msg::SendToLidarMsg>("/send_to_lidar_pack", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVelCallback(msg);
        });
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 5; ++j) {
                enemy_pose_x[i][j] = 0.0f;
                enemy_pose_y[i][j] = 0.0f;
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { this->timerCallback(); });
    }
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        latest_cmd_vel_ = *msg;
        latest_cmd_vel_time_sec_ = this->now().seconds();
    }
    void getSelfPose()
    {
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    
            self_pose_x = transform.transform.translation.x;
            self_pose_y = transform.transform.translation.y;

            tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);

            double yaw, pitch, roll;
            tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
            self_yaw = yaw;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        }
    }

    static constexpr int priority_number_sort[5] = {1, 2, 7, 3, 4}; 

    void cameraCallback(const rm_interfaces::msg::CameraMsg::SharedPtr &msg)
    {
        static const int priority_idx_sort[5] = {0, 1, 4, 2, 3};
        original_camera_msg_ = *msg;
        
        // --- 1. 核心：更新3帧缓存 ---
        // 将当前帧添加到缓存（先移除最旧的，再添加新的）
        buffer_index_ = (buffer_index_ + 1) % 3; // 0->1->2->0 循环
        camera_data_buffer_[buffer_index_] = *msg;
        auto updateTarget = [&](int &id, std::array<float, 2> &enemy_pose_xy, 
                                const std::array<int, 5> &id_list, 
                                const std::array<float, 5> &pose_x, const std::array<float, 5> &pose_y) {
            for (int i = 0; i < 5; ++i) {
                if(id_list[priority_idx_sort[i]] == 0){
                    if(pose_x[priority_idx_sort[i]] > 0.01 && pose_y[priority_idx_sort[i]] > 0.01){
                        id = 0;
                        enemy_pose_xy[0] = 0.0;
                        enemy_pose_xy[1] = 0.0;
                        return;
                    }
                }
                if (id_list[priority_idx_sort[i]] != 0) {
                    id = priority_number_sort[i];   //赋值优先值id
                    enemy_pose_xy[0] = pose_x[priority_idx_sort[i]];
                    enemy_pose_xy[1] = pose_y[priority_idx_sort[i]];
                    return;
                }
            }
            id = 0;
            enemy_pose_xy[0] = 0.0;
            enemy_pose_xy[1] = 0.0;
        };

        for (int i = 0; i < 4; ++i) {
            enemy_pose_x[i] = (i == 0) ? msg->enemy_pose_1_x : (i == 1) ? msg->enemy_pose_2_x : 
                              (i == 2) ? msg->enemy_pose_3_x : msg->enemy_pose_4_x;
            enemy_pose_y[i] = (i == 0) ? msg->enemy_pose_1_y : (i == 1) ? msg->enemy_pose_2_y : 
                              (i == 2) ? msg->enemy_pose_3_y : msg->enemy_pose_4_y;
        }
        
        updateTarget(ids_[0], enemy_pose_xy[0], msg->id_1, msg->enemy_pose_1_x, msg->enemy_pose_1_y); //更新四组目标id和坐标
        updateTarget(ids_[1], enemy_pose_xy[1], msg->id_2, msg->enemy_pose_2_x, msg->enemy_pose_2_y);
        updateTarget(ids_[2], enemy_pose_xy[2], msg->id_3, msg->enemy_pose_3_x, msg->enemy_pose_3_y);
        updateTarget(ids_[3], enemy_pose_xy[3], msg->id_4, msg->enemy_pose_4_x, msg->enemy_pose_4_y);

        // RCLCPP_INFO(this->get_logger(), "id_1_: %d, id_2_: %d, id_3_: %d, id_4_: %d",msg->id_1, msg->id_2, msg->id_3, msg->id_4);
        // RCLCPP_INFO(this->get_logger(), "enemy_pose_1: (%f, %f), enemy_pose_2: (%f, %f)",msg->enemy_pose_1_x, msg->enemy_pose_1_y,msg->enemy_pose_2_x, msg->enemy_pose_2_y);
        // RCLCPP_INFO(this->get_logger(), "enemy_pose_3: (%f, %f), enemy_pose_4: (%f, %f)",msg->enemy_pose_3_x, msg->enemy_pose_3_y,msg->enemy_pose_4_x, msg->enemy_pose_4_y);
    }
    float getFrameTargetZ(const rm_interfaces::msg::CameraMsg& frame, int group_idx)
    {
        static const int priority_idx_sort[5] = {0,1,4,2,3};
        const auto& id_list = (group_idx==0) ? frame.id_1 :
                            (group_idx==1) ? frame.id_2 :
                            (group_idx==2) ? frame.id_3 : frame.id_4;

        const auto& pose_z = (group_idx==0) ? frame.enemy_pose_1_z :
                            (group_idx==1) ? frame.enemy_pose_2_z :
                            (group_idx==2) ? frame.enemy_pose_3_z : frame.enemy_pose_4_z;

        for (int i = 0; i < 5; ++i) {
            int idx = priority_idx_sort[i];
            if (id_list[idx] != 0) {
                return pose_z[idx];
            }
        }
        return 0.0f;
    }

    int getHighestPriorityId()
    { 
        for (int priority_id : priority_number_sort) {
            for (int i = 0; i < 4; ++i) {
                if (ids_[i] == priority_id) {
                    return i; 
                }
            }
        }
        return -1; 
    }
    bool getBigGimbalPose()
    {
        try {

            auto transform = tf_buffer_->lookupTransform(
                aim_odom_frame_, gimbal_frame_, 
                tf2::TimePointZero,  
                std::chrono::milliseconds(200));  

            gimbal_pose_x = transform.transform.translation.x;
            gimbal_pose_y = transform.transform.translation.y;

            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            double pitch, roll;
            tf2::Matrix3x3(q).getEulerYPR(gimbal_yaw, pitch, roll);
            cos_gimbal_yaw_ = cos(-gimbal_yaw);
            sin_gimbal_yaw_ = sin(-gimbal_yaw);
            gimbal_transform_.setOrigin(tf2::Vector3(gimbal_pose_x, gimbal_pose_y, 0.0));
            gimbal_transform_.setRotation(q);

            return true;
        } catch (const tf2::TransformException &ex) {
            // RCLCPP_WARN(this->get_logger(), "TF lookup (map→big_gimbal_link) failed: %s", ex.what());
            return false;
        }
    }
    void convertSpeedToGimbalFrame(double odom_vx, double odom_vy, double &gimbal_vx, double &gimbal_vy)
    {
        // 速度仅需旋转（平移不影响相对速度矢量），复用缓存的旋转参数
        gimbal_vx = odom_vx * cos_gimbal_yaw_ - odom_vy * sin_gimbal_yaw_;
        gimbal_vy = odom_vx * sin_gimbal_yaw_ + odom_vy * cos_gimbal_yaw_;
    }
    void publishGimbalFrameCameraMsg()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 1. 先获取云台位姿，失败则跳过发布
        if (!getBigGimbalPose()) {
            // RCLCPP_WARN(this->get_logger(), "Skip publish gimbal frame msg: get gimbal pose failed");
            return;
        }
        tf2::Transform map_to_gimbal_tf;
        if (!getMapToGimbalTransform(map_to_gimbal_tf)) {
            // RCLCPP_WARN(this->get_logger(), "Skip publish map frame msg: get map->gimbal tf failed");
            return;
        }
        // 2. 初始化新消息
        // rm_interfaces::msg::CameraMsg gimbal_msg;
        // gimbal_msg.header = original_camera_msg_.header;  // 复用原始header
        rm_interfaces::msg::CameraMsg map_msg;
        map_msg.header = original_camera_msg_.header;
        map_msg.header.frame_id = map_frame_;
        auto convertArrayToMap = [&](const std::array<int, 5> & ids,
                                    const std::array<float, 5> & in_x,
                                    const std::array<float, 5> & in_y,
                                    std::array<float, 5> & out_x,
                                    std::array<float, 5> & out_y)
        {
            for (int i = 0; i < 5; ++i) {
                if (ids[i] == 0 || (std::fabs(in_x[i]) < 1e-6 && std::fabs(in_y[i]) < 1e-6)) {
                    out_x[i] = 0.0f;
                    out_y[i] = 0.0f;
                    continue;
                }

                // 先保持你原来的逻辑，得到 local/gimbal/base_link 下坐标
                double local_x = 0.0;
                double local_y = 0.0;
                convertToGimbalFrame(in_x[i], in_y[i], local_x, local_y);
                // std::cout << local_x << "  " << local_y << std::endl;
                // 再用单独读取的 map TF 转成 map 坐标
                double map_x = 0.0;
                double map_y = 0.0;
                convertLocalTargetToMap(map_to_gimbal_tf, local_x, local_y, map_x, map_y);

                out_x[i] = static_cast<float>(map_x);
                out_y[i] = static_cast<float>(map_y);
            }
        };
        // 3. 填充每个相机的目标数据（odom系 + 云台系）
        // 相机1
        // gimbal_msg.id_1 = original_camera_msg_.id_1;
        // gimbal_msg.enemy_pose_1_x = original_camera_msg_.enemy_pose_1_x;
        // gimbal_msg.enemy_pose_1_y = original_camera_msg_.enemy_pose_1_y;
        // for (int i = 0; i < 5; ++i) {
        //     double gimbal_x, gimbal_y;
        //     convertToGimbalFrame(original_camera_msg_.enemy_pose_1_x[i], original_camera_msg_.enemy_pose_1_y[i], gimbal_x, gimbal_y);
        //     gimbal_msg.enemy_pose_1_x[i] = static_cast<float>(gimbal_x);
        //     gimbal_msg.enemy_pose_1_y[i] = static_cast<float>(gimbal_y);
        // }

        // // 相机2
        // gimbal_msg.id_2 = original_camera_msg_.id_2;
        // gimbal_msg.enemy_pose_2_x = original_camera_msg_.enemy_pose_2_x;
        // gimbal_msg.enemy_pose_2_y = original_camera_msg_.enemy_pose_2_y;
        // for (int i = 0; i < 5; ++i) {
        //     double gimbal_x, gimbal_y;
        //     convertToGimbalFrame(original_camera_msg_.enemy_pose_2_x[i], original_camera_msg_.enemy_pose_2_y[i], gimbal_x, gimbal_y);
        //     gimbal_msg.enemy_pose_2_x[i] = static_cast<float>(gimbal_x);
        //     gimbal_msg.enemy_pose_2_y[i] = static_cast<float>(gimbal_y);
        // }

        // // 相机3
        // gimbal_msg.id_3 = original_camera_msg_.id_3;
        // gimbal_msg.enemy_pose_3_x = original_camera_msg_.enemy_pose_3_x;
        // gimbal_msg.enemy_pose_3_y = original_camera_msg_.enemy_pose_3_y;
        // for (int i = 0; i < 5; ++i) {
        //     double gimbal_x, gimbal_y;
        //     convertToGimbalFrame(original_camera_msg_.enemy_pose_3_x[i], original_camera_msg_.enemy_pose_3_y[i], gimbal_x, gimbal_y);
        //     gimbal_msg.enemy_pose_3_x[i] = static_cast<float>(gimbal_x);
        //     gimbal_msg.enemy_pose_3_y[i] = static_cast<float>(gimbal_y);
        // }

        // // 相机4
        // gimbal_msg.id_4 = original_camera_msg_.id_4;
        // gimbal_msg.enemy_pose_4_x = original_camera_msg_.enemy_pose_4_x;
        // gimbal_msg.enemy_pose_4_y = original_camera_msg_.enemy_pose_4_y;
        // for (int i = 0; i < 5; ++i) {
        //     double gimbal_x, gimbal_y;
        //     convertToGimbalFrame(original_camera_msg_.enemy_pose_4_x[i], original_camera_msg_.enemy_pose_4_y[i], gimbal_x, gimbal_y);
        //     gimbal_msg.enemy_pose_4_x[i] = static_cast<float>(gimbal_x);
        //     gimbal_msg.enemy_pose_4_y[i] = static_cast<float>(gimbal_y);
        // }
        map_msg.id_1 = original_camera_msg_.id_1;
        convertArrayToMap(
            original_camera_msg_.id_1,
            original_camera_msg_.enemy_pose_1_x,
            original_camera_msg_.enemy_pose_1_y,
            map_msg.enemy_pose_1_x,
            map_msg.enemy_pose_1_y);
    
        map_msg.id_2 = original_camera_msg_.id_2;
        convertArrayToMap(
            original_camera_msg_.id_2,
            original_camera_msg_.enemy_pose_2_x,
            original_camera_msg_.enemy_pose_2_y,
            map_msg.enemy_pose_2_x,
            map_msg.enemy_pose_2_y);
    
        map_msg.id_3 = original_camera_msg_.id_3;
        convertArrayToMap(
            original_camera_msg_.id_3,
            original_camera_msg_.enemy_pose_3_x,
            original_camera_msg_.enemy_pose_3_y,
            map_msg.enemy_pose_3_x,
            map_msg.enemy_pose_3_y);
    
        map_msg.id_4 = original_camera_msg_.id_4;
        convertArrayToMap(
            original_camera_msg_.id_4,
            original_camera_msg_.enemy_pose_4_x,
            original_camera_msg_.enemy_pose_4_y,
            map_msg.enemy_pose_4_x,
            map_msg.enemy_pose_4_y);
    
        map_msg.speed_x = original_camera_msg_.speed_x;
        map_msg.speed_y = original_camera_msg_.speed_y;
    
        gimbal_camera_pub_->publish(map_msg);
    }
    //     4. 填充速度数据（保留原始odom系，若需转换可新增云台系速度）
    //     gimbal_msg.speed_x = original_camera_msg_.speed_x;
    //     gimbal_msg.speed_y = original_camera_msg_.speed_y;

    //     // 5. 发布新话题
    //     gimbal_camera_pub_->publish(gimbal_msg);
    //     RCLCPP_DEBUG(this->get_logger(), "Published gimbal frame camera msg (topic: /gimbal_frame_camera_topic)");
    // }
    //  void publishGimbalFrameCameraMsg()
    // {
    //     rm_interfaces::msg::CameraMsg zero_msg;

    //     // 填充消息头，保持时间戳与坐标系兼容
    //     zero_msg.header.stamp = this->now();
    //     zero_msg.header.frame_id = map_frame_;

    //     // 所有ID数组置零
    //     std::fill(zero_msg.id_1.begin(), zero_msg.id_1.end(), 0);
    //     std::fill(zero_msg.id_2.begin(), zero_msg.id_2.end(), 0);
    //     std::fill(zero_msg.id_3.begin(), zero_msg.id_3.end(), 0);
    //     std::fill(zero_msg.id_4.begin(), zero_msg.id_4.end(), 0);

    //     // 相机1位置全零
    //     std::fill(zero_msg.enemy_pose_1_x.begin(), zero_msg.enemy_pose_1_x.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_1_y.begin(), zero_msg.enemy_pose_1_y.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_1_z.begin(), zero_msg.enemy_pose_1_z.end(), 0.0f);

    //     // 相机2位置全零
    //     std::fill(zero_msg.enemy_pose_2_x.begin(), zero_msg.enemy_pose_2_x.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_2_y.begin(), zero_msg.enemy_pose_2_y.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_2_z.begin(), zero_msg.enemy_pose_2_z.end(), 0.0f);

    //     // 相机3位置全零
    //     std::fill(zero_msg.enemy_pose_3_x.begin(), zero_msg.enemy_pose_3_x.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_3_y.begin(), zero_msg.enemy_pose_3_y.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_3_z.begin(), zero_msg.enemy_pose_3_z.end(), 0.0f);

    //     // 相机4位置全零
    //     std::fill(zero_msg.enemy_pose_4_x.begin(), zero_msg.enemy_pose_4_x.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_4_y.begin(), zero_msg.enemy_pose_4_y.end(), 0.0f);
    //     std::fill(zero_msg.enemy_pose_4_z.begin(), zero_msg.enemy_pose_4_z.end(), 0.0f);

    //     // 速度数组全零
    //     std::fill(zero_msg.speed_x.begin(), zero_msg.speed_x.end(), 0.0f);
    //     std::fill(zero_msg.speed_y.begin(), zero_msg.speed_y.end(), 0.0f);

    //     gimbal_camera_pub_->publish(zero_msg);
    // }
    void convertToGimbalFrame(double odom_x, double odom_y, double &gimbal_x, double &gimbal_y)
    {
        
        tf2::Vector3 target_map(odom_x, odom_y, 0.0);
        
        tf2::Vector3 target_gimbal = gimbal_transform_.inverse() * target_map;
        
        gimbal_x = target_gimbal.x();
        gimbal_y = target_gimbal.y();
    }
    // void convertToGimbalFrame(double car_x, double car_y, double &gimbal_x, double &gimbal_y)
    // {
    // gimbal_x = car_x * cos_gimbal_yaw_ - car_y * sin_gimbal_yaw_;
    // gimbal_y = car_x * sin_gimbal_yaw_ + car_y * cos_gimbal_yaw_;
    // }
    std::pair<float, float> getFrameTargetPose(const rm_interfaces::msg::CameraMsg& frame, int target_group_idx) {
        static const int priority_idx_sort[5] = {0, 1, 4, 2, 3};
        static constexpr int priority_number_sort[5] = {1, 2, 7, 3, 4};
        
        // 根据目标组索引获取对应ID列表和坐标列表
        const auto& id_list = (target_group_idx == 0) ? frame.id_1 :
                              (target_group_idx == 1) ? frame.id_2 :
                              (target_group_idx == 2) ? frame.id_3 : frame.id_4;
        const auto& pose_x = (target_group_idx == 0) ? frame.enemy_pose_1_x :
                             (target_group_idx == 1) ? frame.enemy_pose_2_x :
                             (target_group_idx == 2) ? frame.enemy_pose_3_x : frame.enemy_pose_4_x;
        const auto& pose_y = (target_group_idx == 0) ? frame.enemy_pose_1_y :
                             (target_group_idx == 1) ? frame.enemy_pose_2_y :
                             (target_group_idx == 2) ? frame.enemy_pose_3_y : frame.enemy_pose_4_y;

        // 按优先级顺序查找有效坐标
        for (int i = 0; i < 5; ++i) {
            int idx = priority_idx_sort[i];
            if (id_list[idx] != 0 ) {
                return {pose_x[idx], pose_y[idx]};
            }
        }
        return {0.0f, 0.0f}; // 无有效坐标返回(0,0)
    }
    static int getPriorityLevel(int priority_id) {
        static constexpr int priority_number_sort[5] = {1, 7, 3, 4 ,2};
        for (int i = 0; i < 5; ++i) {
            if (priority_number_sort[i] == priority_id) {
                return i; // 0=最高优先级，4=最低优先级
            }
        }
        return 999; // 无效ID返回极低优先级
    }
    std::array<rm_interfaces::msg::CameraMsg, 3> getLast3Frames() const {
        std::array<rm_interfaces::msg::CameraMsg, 3> result;
        if (buffer_index_ == -1) { // 还未收到任何帧
            return result;
        }
        // 按时间顺序：最旧帧 -> 次旧帧 -> 最新帧
        result[0] = camera_data_buffer_[(buffer_index_ + 1) % 3];
        result[1] = camera_data_buffer_[(buffer_index_ + 2) % 3];
        result[2] = camera_data_buffer_[buffer_index_]; // 最新帧
        return result;
    }
    bool getMapToGimbalTransform(tf2::Transform &map_to_gimbal_tf)
    {
        try {
            auto tf_msg = tf_buffer_->lookupTransform(
                map_frame_,
                "base_link",
                tf2::TimePointZero,
                std::chrono::milliseconds(200));

            tf2::Quaternion q(
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w);
            
            tf2::Vector3 t(
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z);
            
            map_to_gimbal_tf.setOrigin(t);
            map_to_gimbal_tf.setRotation(q);
            return true;

        } catch (const tf2::TransformException &ex) {
            // RCLCPP_WARN(
            //     this->get_logger(),
            //     "TF lookup (%s -> %s) failed: %s",
            //     map_frame_.c_str(),
            //     gimbal_frame_.c_str(),
            //     ex.what());
            return false;
        }
    }
    void convertLocalTargetToMap(
        const tf2::Transform &map_to_local_tf,
        double local_x,
        double local_y,
        double &map_x,
        double &map_y)
    {
        tf2::Vector3 target_local(local_x, local_y, 0.0);
        tf2::Vector3 target_map = map_to_local_tf * target_local;
    
        map_x = target_map.x();
        map_y = target_map.y();
    }
   std::tuple<int, std::pair<float, float>, int> getFrameHighestPriorityTarget(const rm_interfaces::msg::CameraMsg& frame) {
        static constexpr int priority_number_sort[5] = {7, 1, 2, 3, 4};
        int highest_priority_id = -1;
        std::pair<float, float> highest_priority_pose = {0.0f, 0.0f};
        int highest_level = 999;
        int target_camera_group = -1; // 新增：记录目标所属相机组（0=id_1,1=id_2,2=id_3,3=id_4）

        // 遍历4个目标组
        for (int group_idx = 0; group_idx < 4; ++group_idx) {
            auto [pose_x, pose_y] = getFrameTargetPose(frame, group_idx);

            // 获取该目标的优先级ID
            int priority_id = 0;
            static const int priority_idx_sort[5] = {0, 1, 4, 2, 3};
            const auto& id_list = (group_idx == 0) ? frame.id_1 :
                                  (group_idx == 1) ? frame.id_2 :
                                  (group_idx == 2) ? frame.id_3 : frame.id_4;
            for (int i = 0; i < 5; ++i) {
                int idx = priority_idx_sort[i];
                if (id_list[idx] != 0) {
                    priority_id = priority_number_sort[i];
                    break;
                }
            }
            if (priority_id == 0) {
                continue;
            }

            // 比较优先级，保留最高的
            int current_level = getPriorityLevel(priority_id);
            if (current_level < highest_level) {
                highest_level = current_level;
                highest_priority_id = priority_id;
                highest_priority_pose = {pose_x, pose_y};
                target_camera_group = group_idx; // 记录所属相机组
            }
        }

        return {highest_priority_id, highest_priority_pose, target_camera_group};
    }
    void clearLast3Frames() {
        buffer_index_ = -1;
       for (auto& frame : camera_data_buffer_) {
            // ========== 清空所有ID数组 ==========
            std::fill(frame.id_1.begin(), frame.id_1.end(), 0);
            std::fill(frame.id_2.begin(), frame.id_2.end(), 0);
            std::fill(frame.id_3.begin(), frame.id_3.end(), 0);
            std::fill(frame.id_4.begin(), frame.id_4.end(), 0);

            // ========== 清空所有位置数组 ==========
            std::fill(frame.enemy_pose_1_x.begin(), frame.enemy_pose_1_x.end(), 0.0f);
            std::fill(frame.enemy_pose_1_y.begin(), frame.enemy_pose_1_y.end(), 0.0f);
            std::fill(frame.enemy_pose_2_x.begin(), frame.enemy_pose_2_x.end(), 0.0f);
            std::fill(frame.enemy_pose_2_y.begin(), frame.enemy_pose_2_y.end(), 0.0f);
            std::fill(frame.enemy_pose_3_x.begin(), frame.enemy_pose_3_x.end(), 0.0f);
            std::fill(frame.enemy_pose_3_y.begin(), frame.enemy_pose_3_y.end(), 0.0f);
            std::fill(frame.enemy_pose_4_x.begin(), frame.enemy_pose_4_x.end(), 0.0f);
            std::fill(frame.enemy_pose_4_y.begin(), frame.enemy_pose_4_y.end(), 0.0f);

            // ========== 清空速度数组 ==========
            std::fill(frame.speed_x.begin(), frame.speed_x.end(), 0.0f);
            std::fill(frame.speed_y.begin(), frame.speed_y.end(), 0.0f);

            // ========== 清空header（标记帧无效） ==========
            frame.header.stamp = rclcpp::Time(0);
            frame.header.frame_id = "";
        }
        RCLCPP_DEBUG(this->get_logger(), "Cleared last 3 frames cache successfully");
    }
    void calculateAngle() { 
        std::lock_guard<std::mutex> lock(data_mutex_);
        double angle_deg = 0.0;
        double yaw_deg = 0.0;
        double pitch_deg = 0.0;
        // --- 获取3帧缓存 ---
        auto last_3_frames = getLast3Frames();
        int valid_frame_count = 0;
        int global_highest_priority_id = -1;        // 全局最高优先级ID
        int global_highest_level = 999;             // 全局最高优先级等级（0最高）
        float target_x = 0.0f, target_y = 0.0f,target_z = 0.0f;     // 最后出现帧的目标坐标
        int target_camera_group = -1;               // 目标所属相机组
        int camera0_frame_count = 0;                // 统计：3帧中有主相机数据的帧数

        for (int frame_idx = 2; frame_idx >= 0; --frame_idx) {
            const auto& frame = last_3_frames[frame_idx];
            auto [frame_priority_id, frame_pose, frame_group] = getFrameHighestPriorityTarget(frame);
            float frame_target_x = frame_pose.first;
            float frame_target_y = frame_pose.second;
            float frame_target_z = getFrameTargetZ(frame, frame_group);

            // 统计有效帧数量
            if (frame_priority_id != -1 && frame_priority_id != 0) {
                valid_frame_count++;
            }
            if (frame_group == 0 && frame_priority_id != -1 && frame_priority_id != 0) {
                camera0_frame_count++;
            }
            if (frame_priority_id == -1 || frame_priority_id == 0 || (frame_target_x == 0.0f && frame_target_y == 0.0f)) {
                continue;
            }
    
            int frame_level = getPriorityLevel(frame_priority_id);
            bool is_higher_priority = (frame_level < global_highest_level);
            bool is_same_priority = (frame_level == global_highest_level);
    
            // 缓存目标数据：
            // - 优先级更高 → 直接更新
            // - 优先级相同 → 保留最后出现的帧（反向遍历，后遍历到的帧更旧，不更新）
            if (is_higher_priority) {
                global_highest_priority_id = frame_priority_id;
                global_highest_level = frame_level;
                target_x = frame_target_x;
                target_y = frame_target_y;
                target_z = frame_target_z;
                target_camera_group = frame_group;
            }
        }
    
        // 有效帧不足2帧 or 主相机有两帧以上 → 发布0角度
        if (valid_frame_count < 2 || camera0_frame_count > 1) {
            geometry_msgs::msg::Point32 msg;
            msg.x = 0.0f;
            msg.y = 0.0f;
            msg.z = 0.0f;
            angle_pub_->publish(msg);
            return;
        }
       if (target_camera_group == 0) {
            RCLCPP_INFO(this->get_logger(), 
                        "Target from id_1 camera, skip publish angle (priority ID: %d)",
                        global_highest_priority_id);
            // 可选：发布0角度 或 直接返回
            geometry_msgs::msg::Point32 msg;
            msg.x = 0.0f;
            msg.y = 0.0f;
            msg.z = 0.0f;
            angle_pub_->publish(msg);
            return;
        }
        if (global_highest_priority_id == -1) {
            RCLCPP_WARN(this->get_logger(), "No valid target in latest frame, publish 0 angle");
            geometry_msgs::msg::Point32 msg;
            msg.x = 0.0f;
            msg.y = 0.0f;
            msg.z = 0.0f;
            angle_pub_->publish(msg);
            return;
        }

        // --- 获取云台位姿 ---
        if (!getBigGimbalPose()) {
            // RCLCPP_WARN(this->get_logger(), "Failed to get big_gimbal_link pose, publish 0 angle");
            // rm_interfaces::msg::AngleMsg msg;
            // msg.angle = 0.0;
            // angle_pub_->publish(msg);
            return;
        }

        // --- 转换目标坐标到云台系 ---
        double target_gimbal_x, target_gimbal_y;
        convertToGimbalFrame(target_x, target_y, target_gimbal_x, target_gimbal_y);
        // compensateTargetInGimbalFrame(target_gimbal_x, target_gimbal_y);
        // --- 计算角度（使用最后一帧数据）---
        // double angle_rad = atan2(target_gimbal_y, target_gimbal_x);
        double yaw_rad = atan2(target_gimbal_y, target_gimbal_x);
        yaw_deg = yaw_rad * 180.0 / M_PI + 3.0;
        // angle_deg = angle_rad * 180.0 / M_PI + 3.0; // 补
        double dist_2d = std::hypot(target_gimbal_x, target_gimbal_y);
        if (dist_2d > 1e-6)
        {
            double pitch_rad = atan2(target_z, dist_2d);
            pitch_deg = pitch_rad * 180.0 / M_PI;
        }
        else
        {
            pitch_deg = 0.0;
        }


        RCLCPP_INFO(this->get_logger(), 
                    "Valid frames: %d, Latest frame highest priority ID: %d, pose: (%.2f, %.2f), angle: %.2f degrees",
                    valid_frame_count, global_highest_priority_id, target_x, target_y, angle_deg);
        
        geometry_msgs::msg::Point32 msg;
        msg.x = static_cast<float>(yaw_deg);   // x = yaw
        msg.y = static_cast<float>(pitch_deg); // y = pitch
        msg.z = 0.0f;
        angle_pub_->publish(msg);
        // clearLast3Frames(); 
    }
    void convertGimbalFrameToMap(double gimbal_x, double gimbal_y, double &map_x, double &map_y)
    {
        tf2::Vector3 target_gimbal(gimbal_x, gimbal_y, 0.0);

        // gimbal_transform_ 是 aim_odom_frame_ -> gimbal_frame_ 的变换
        // 所以它可以把 gimbal_frame 下的点变到 aim_odom_frame_ 下
        tf2::Vector3 target_map = gimbal_transform_ * target_gimbal;

        map_x = target_map.x();
        map_y = target_map.y();
    }
    void compensateTargetInGimbalFrame(double &target_gimbal_x, double &target_gimbal_y)
    {
        if (!enable_motion_compensation_) {
            return;
        }
    
        geometry_msgs::msg::Twist cmd_vel;
        double cmd_age = 999.0;
    
        {
            std::lock_guard<std::mutex> lock(vel_mutex_);
            cmd_vel = latest_cmd_vel_;
    
            if (latest_cmd_vel_time_sec_ > 0.0) {
                cmd_age = this->now().seconds() - latest_cmd_vel_time_sec_;
            }
        }
    
        if (cmd_age > cmd_vel_timeout_) {
            return;
        }
    
        const double vx_base = cmd_vel.linear.x;
        const double vy_base = cmd_vel.linear.y;
    
        const double dx_base = vx_base * aim_delay_;
        const double dy_base = vy_base * aim_delay_;
    
        target_gimbal_x -= dx_base;
        target_gimbal_y -= dy_base;
    }
    void PoseToLidar()
    { 
        getSelfPose();
        // RCLCPP_INFO(this->get_logger(), "%d", red_blue);

        if (red_blue)
        {
            for (int i = 0; i < 4; ++i) 
            {
                for (int j = 0; j < 5; ++j) {
                    if (enemy_pose_x[i][j] != 0.0f && enemy_pose_y[i][j] != 0.0f) {  //如果存在目标
                        // 应用 yaw 旋转
                        float cos_yaw = cos(self_yaw);
                        float sin_yaw = sin(self_yaw);

                        float rotated_x = enemy_pose_x[i][j] * cos_yaw - enemy_pose_y[i][j] * sin_yaw;
                        float rotated_y = enemy_pose_x[i][j] * sin_yaw + enemy_pose_y[i][j] * cos_yaw;

                        enemy_poses_x_to_lidar[j] = rotated_x + self_pose_x + transform_red_x_;
                        enemy_poses_y_to_lidar[j] = rotated_y + self_pose_y + transform_red_y_; 

                        // RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", enemy_pose_x[i][j], self_pose_x, transform_red_x_);
                        // RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", enemy_pose_y[i][j], self_pose_y, transform_red_y_);
                    }
                }
            }  
        }
        else
        {
            for (int i = 0; i < 4; ++i) 
            {
                for (int j = 0; j < 5; ++j) {
                    if (enemy_pose_x[i][j] != 0.0f && enemy_pose_y[i][j] != 0.0f) {
                        // 应用 yaw 旋转
                        float cos_yaw = cos(self_yaw);
                        float sin_yaw = sin(self_yaw);

                        float rotated_x = enemy_pose_x[i][j] * cos_yaw - enemy_pose_y[i][j] * sin_yaw;
                        float rotated_y = enemy_pose_x[i][j] * sin_yaw + enemy_pose_y[i][j] * cos_yaw;

                        enemy_poses_x_to_lidar[j] = transform_blue_x_ - rotated_x - self_pose_x;
                        enemy_poses_y_to_lidar[j] = transform_blue_y_ - rotated_y - self_pose_y; 

                        // RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", enemy_pose_x[i][j], self_pose_x, transform_blue_x_);
                        // RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", enemy_pose_y[i][j], self_pose_y, transform_blue_y_);
                    }
                }
            }  
        }
    
    rm_interfaces::msg::SendToLidarMsg msg;
    msg.enemy_poses_x_to_lidar = enemy_poses_x_to_lidar;
    msg.enemy_poses_y_to_lidar = enemy_poses_y_to_lidar;
    send_to_lidar_pub_->publish(msg);

    std::fill(enemy_poses_x_to_lidar.begin(), enemy_poses_x_to_lidar.end(), 0.0f);
    std::fill(enemy_poses_y_to_lidar.begin(), enemy_poses_y_to_lidar.end(), 0.0f);

    }
    
    void timerCallback()
    {
        calculateAngle();
        // PoseToLidar();
        publishGimbalFrameCameraMsg();
    }

private:
    rclcpp::Subscription<rm_interfaces::msg::CameraMsg>::SharedPtr camera_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr angle_pub_;
    rclcpp::Publisher<rm_interfaces::msg::SendToLidarMsg>::SharedPtr send_to_lidar_pub_;
    rclcpp::Publisher<rm_interfaces::msg::CameraMsg>::SharedPtr gimbal_camera_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rm_interfaces::msg::CameraMsg original_camera_msg_;
    std::mutex data_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;

    double self_pose_x;
    double self_pose_y;
    double self_yaw;
    std::string map_frame_ = "map";
    tf2::Transform gimbal_transform_;
    double gimbal_pose_x = 0.0;
    double gimbal_pose_y = 0.0;
    double gimbal_yaw = 0.0;
    double cos_gimbal_yaw_ = 1.0;
    double sin_gimbal_yaw_ = 0.0;
    int ids_[4] = {0, 0, 0, 0}; 

    std::array<float, 5> enemy_poses_x_to_lidar, enemy_poses_y_to_lidar;
    std::array<std::array<float, 2>, 4> enemy_pose_xy;
    std::array<std::array<float, 5>, 4> enemy_pose_x, enemy_pose_y;
    
    double transform_red_x_;
    double transform_red_y_;
    double transform_blue_x_;
    double transform_blue_y_;
    bool red_blue;

    std::string aim_odom_frame_;
    std::string gimbal_frame_;

    std::array<rm_interfaces::msg::CameraMsg, 3> camera_data_buffer_; // 存储3帧数据的数组
    int buffer_index_ = -1;   
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    std::mutex vel_mutex_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    double latest_cmd_vel_time_sec_ = -1.0;

    std::string cmd_vel_topic_;
    double motion_comp_delay_ = 0.15;
    double cmd_vel_timeout_ = 0.2;
    bool enable_motion_compensation_ = true;
    double aim_delay_ = 0.4;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniPerception>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}