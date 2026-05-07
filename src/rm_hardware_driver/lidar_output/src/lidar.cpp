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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "livox_ros_driver2/msg/custom_msg.hpp"

using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomPoint = livox_ros_driver2::msg::CustomPoint;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<LivoxCustomMsg, LivoxCustomMsg>;

class lidar_output : public rclcpp::Node
{
public:
    lidar_output() : Node("lidar_output")
    {
        this->declare_parameter<std::string>("topic1", "livox/lidar_192_168_2_120");
        this->declare_parameter<std::string>("topic2", "livox/lidar_192_168_2_121");
        this->declare_parameter<std::string>("target_frame_id", "lidar_link");
        this->declare_parameter<double>("sync_max_delay", 0.001);
        
        this->get_parameter("topic1", topic_1);
        this->get_parameter("topic2", topic_2);
        this->get_parameter("target_frame_id", target_frame_id_);
        this->get_parameter("sync_max_delay", sync_max_delay_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub1_.subscribe(this, topic_1);
        sub2_.subscribe(this, topic_2);
        
        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(10),
            sub1_, sub2_
        );
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_delay_));
        sync_->registerCallback(std::bind(&lidar_output::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_merged_cloud_ = this->create_publisher<LivoxCustomMsg>("/livox/lidar", 20);

        RCLCPP_INFO(this->get_logger(), "Lidar Output Node Started");
    }

private:

// ==========================
// 雷达1 过滤参数（你要的窄范围）
// ==========================
const float L1_MIN_DIST = 0.1f;
const float L1_X_MIN = -0.2f;    
const float L1_X_MAX = 0.2f;  
const float L1_Y_MIN = -0.4f;
const float L1_Y_MAX =  0.1f;
const float L1_Z_MIN = -0.2f;
const float L1_Z_MAX =  0.1f;

// ==========================
// 雷达2 过滤参数（你要的窄范围）
// ==========================
const float L2_MIN_DIST = 0.1f;
const float L2_X_MIN = -0.2f;    
const float L2_X_MAX = 0.2f;  
const float L2_Y_MIN = -0.1f;
const float L2_Y_MAX =  0.4f;
const float L2_Z_MIN = -0.2f;
const float L2_Z_MAX =  0.1f;

// ==========================
// 雷达1 正确过滤（你要的方向！）
// ==========================
bool filter1(const LivoxCustomPoint& p)
{
    // 距离太近 → 丢掉
    float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
    if (d2 < L1_MIN_DIST)
        return false;
    // Y 超出范围 → 丢掉
    if (p.x > L1_X_MIN && p.x < L1_X_MAX && p.y < L1_Y_MAX && p.y > L1_Y_MIN && p.z < L1_Z_MAX && p.z > L1_Z_MIN)
        return false;

    // Z 超出范围 → 丢掉
    // if (p.z < L1_Z_MAX && p.z > L1_Z_MIN)
    //     return false;

    // 剩下的都保留
    return true;
}

// ==========================
// 雷达2 正确过滤（你要的方向！）
// ==========================
bool filter2(const LivoxCustomPoint& p)
{
    float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
    if (d2 < L2_MIN_DIST )
        return false;
    if (p.x > L2_X_MIN && p.x < L2_X_MAX && p.y < L2_Y_MAX && p.y > L2_Y_MIN && p.z < L2_Z_MAX && p.z > L2_Z_MIN)
        return false;

    return true;
}
    void sync_callback(const LivoxCustomMsg::ConstSharedPtr& msg1,
                       const LivoxCustomMsg::ConstSharedPtr& msg2)
    {
        try {
            LivoxCustomMsg lidar_msg;
            lidar_msg.header = msg1->header;
            lidar_msg.header.frame_id = target_frame_id_;
            lidar_msg.timebase = msg1->timebase;
            lidar_msg.lidar_id = msg1->lidar_id;
            lidar_msg.point_num = 0;
            lidar_msg.points.clear();

            // ------------------------------
            // 雷达1：先过滤再加入
            // ------------------------------
            for (const auto& p : msg1->points) {
                if (filter1(p)) {
                    lidar_msg.points.push_back(p);
                    lidar_msg.point_num++;
                }
            }

            // ------------------------------
            // TF 初始化
            // ------------------------------
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

            // ------------------------------
            // 雷达2：先过滤 → 再变换
            // ------------------------------
            for (const auto& src_point : msg2->points) {
                // 过滤！
                if (!filter2(src_point)) continue;

                LivoxCustomPoint dst_point = src_point;
                Eigen::Vector3d pt(src_point.x, src_point.y, src_point.z);
                Eigen::Vector3d pt_transformed = transform * pt;
                dst_point.x = pt_transformed.x();
                dst_point.y = pt_transformed.y();
                dst_point.z = pt_transformed.z();

                lidar_msg.points.push_back(dst_point);
                lidar_msg.point_num++;
            }

            // ------------------------------
            // 一定发布！
            // ------------------------------
            if (lidar_msg.point_num > 0) {
                pub_merged_cloud_->publish(lidar_msg);
            }

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        } catch (std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", ex.what());
        }
    }

    message_filters::Subscriber<LivoxCustomMsg> sub1_;
    message_filters::Subscriber<LivoxCustomMsg> sub2_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
    rclcpp::Publisher<LivoxCustomMsg>::SharedPtr pub_merged_cloud_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_id_;
    bool is_init = false;
    Eigen::Affine3d transform;

    std::string topic_1, topic_2;
    double sync_max_delay_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_output>());
    rclcpp::shutdown();
    return 0;
}