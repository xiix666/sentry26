#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_types.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// 引入message_filters实现时间同步
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Livox驱动2的自定义消息
#include "livox_ros_driver2/msg/custom_msg.hpp"

// 类型别名简化代码
using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomPoint = livox_ros_driver2::msg::CustomPoint;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
// 近似时间同步策略（适配Livox点云时间戳微小差异）
using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<LivoxCustomMsg, LivoxCustomMsg>;

class lidar_output : public rclcpp::Node
{
public:
    lidar_output() : Node("lidar_output")
    {
        // ========== 1. 声明并读取参数 ==========
        this->declare_parameter<std::string>("topic1", "livox/lidar_192_168_2_120");
        this->declare_parameter<std::string>("topic2", "livox/lidar_192_168_2_121");
        // 新增：目标坐标系（默认转换到第一个lidar的坐标系）
        this->declare_parameter<std::string>("target_frame_id", "lidar_link");
        // 新增：时间同步最大延迟（秒，默认10ms）
        this->declare_parameter<double>("sync_max_delay", 0.001);
        
        this->get_parameter("topic1", topic_1);
        this->get_parameter("topic2", topic_2);
        this->get_parameter("target_frame_id", target_frame_id_);
        this->get_parameter("sync_max_delay", sync_max_delay_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ========== 3. 初始化消息订阅器和时间同步器 ==========
        // 用message_filters的Subscriber替代原生订阅器（用于同步）
        sub1_.subscribe(this, topic_1);
        sub2_.subscribe(this, topic_2);
        
        // 初始化近似时间同步器
        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(10),  // 同步队列大小
            sub1_, sub2_            // 要同步的两个订阅器
        );
        // 设置同步最大延迟
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_delay_));
        // 注册同步回调函数
        sync_->registerCallback(std::bind(&lidar_output::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        // ========== 4. 初始化发布器（发布合并后的点云） ==========
        pub_merged_cloud_ = this->create_publisher<LivoxCustomMsg>("/livox/lidar", 20);

        RCLCPP_INFO(this->get_logger(), "Lidar Output Node Started:");
        RCLCPP_INFO(this->get_logger(), "  - Topic 1: %s", topic_1.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Topic 2: %s", topic_2.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Target Frame: %s", target_frame_id_.c_str());
    }

private:
    // ========== 核心回调：处理同步后的两个Livox点云 ==========
    void sync_callback(const LivoxCustomMsg::ConstSharedPtr& msg1,
                       const LivoxCustomMsg::ConstSharedPtr& msg2)
    {
        try {
            LivoxCustomMsg lidar_msg;
            lidar_msg.header = msg1->header;
            lidar_msg.header.frame_id = target_frame_id_;
            lidar_msg.timebase = msg1->timebase;
            lidar_msg.lidar_id = msg1->lidar_id;
            lidar_msg.points = msg1->points;
            lidar_msg.point_num = msg1->point_num;
            // std::cout << target_frame_id_ << "  " << msg2->header.frame_id << std::endl;
            if(!is_init){
                geometry_msgs::msg::TransformStamped transform_stamp = 
                tf_buffer_->lookupTransform(
                    target_frame_id_,
                    msg2->header.frame_id,    
                    
                    tf2::TimePointZero
                );
                transform = tf2::transformToEigen(transform_stamp.transform);
                is_init = true;
            }

            // std::cout << "1111" << std::endl;
            // TF转Eigen变换矩阵
            
            // std::cout << transform.matrix() << std::endl;
            for (const auto& src_point : msg2->points) {
                LivoxCustomPoint dst_point = src_point;  // 复制所有原生字段（offset_time/tag/line等）
                
                // 转换坐标（x/y/z从msg2坐标系→目标坐标系，单位：m）
                Eigen::Vector3d pt(src_point.x, src_point.y, src_point.z);
                Eigen::Vector3d pt_transformed = transform * pt;
                dst_point.x = pt_transformed.x();
                dst_point.y = pt_transformed.y();
                dst_point.z = pt_transformed.z();
                
                float dist = src_point.x*src_point.x +src_point.y*src_point.y+ src_point.z*src_point.z;
                if(dist < 0.1) {continue;}
                // 将转换后的点加入合并数组
                lidar_msg.points.push_back(dst_point);
                lidar_msg.point_num++;  // 累计点云数量
            }
            pub_merged_cloud_->publish(lidar_msg);

        } catch (tf2::TransformException& ex) {
            // TF查找失败（如静态TF未发布）
            RCLCPP_WARN(this->get_logger(), "TF Transform Error: %s", ex.what());
        } catch (std::exception& ex) {
            // 其他异常
            RCLCPP_ERROR(this->get_logger(), "Processing Error: %s", ex.what());
        }
    }

    // ========== 成员变量 ==========
    // 订阅器（message_filters版本，用于同步）
    message_filters::Subscriber<LivoxCustomMsg> sub1_;
    message_filters::Subscriber<LivoxCustomMsg> sub2_;
    // 时间同步器
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
    // 发布器
    rclcpp::Publisher<LivoxCustomMsg>::SharedPtr pub_merged_cloud_;

    // TF相关

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_id_;  // 目标坐标系
    bool is_init = false;
    Eigen::Affine3d transform;
    // 配置参数
    std::string topic_1, topic_2;
    double sync_max_delay_;        // 时间同步最大延迟
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // 自旋运行节点
    rclcpp::spin(std::make_shared<lidar_output>());
    rclcpp::shutdown();
    return 0;
}