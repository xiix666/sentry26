#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>

class PointCloudFrameTrans : public rclcpp::Node
{
public:
    PointCloudFrameTrans()
        : Node("pointcloud_frame_trans")
    {
        // 从参数服务器获取参数
        this->declare_parameter("input_topic", "/livox/pointcloud2");
        this->declare_parameter("output_topic", "/livox/pointcloud2/transframe");
        this->declare_parameter("output_frame", "odom");
        this->declare_parameter("tran_odom", false);
        this->declare_parameter("odom_in", "aft_mapped_to_init");
        this->declare_parameter("odom_out", "lidar_odometry");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("lidar_frame", "lidar_link");
        this->declare_parameter("odom_base_out", "base_odometry");

        this->declare_parameter("limit_x", 10.0); // X轴方向总长度
        this->declare_parameter("limit_y", 10.0); // Y轴方向总长度
        this->declare_parameter("limit_z", 3.0);  // Z轴高度限制 (顺便滤掉地面和天空)
        // this->declare_parameter("enable_voxel", true); // 是否开启体素滤波进一步降采样

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("output_frame", output_frame_);
        this->get_parameter("tran_odom", tran_odom_);
        this->get_parameter("odom_in", odom_in_);
        this->get_parameter("odom_out", odom_out_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("lidar_frame", lidar_frame_);
        this->get_parameter("odom_base_out", odom_base_out_);

        this->get_parameter("limit_x", limit_x_);
        this->get_parameter("limit_y", limit_y_);
        this->get_parameter("limit_z", limit_z_);
        // this->get_parameter("enable_voxel", enable_voxel_);
        // 初始化tf监听器，tf_buffer将被填充
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(5));
        odom_qos.reliable();
        odom_qos.durability_volatile();
        // 初始化发布者
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 5);
        if (tran_odom_) {
            pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_out_, 15);
            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_in_, odom_qos, std::bind(&PointCloudFrameTrans::odomCallback, this, std::placeholders::_1));
        }
        pub_base_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_base_out_, 5); // odom -> base
        std::cout << "tran节点启动" << std::endl;
        // 订阅原始点云消息
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 15, std::bind(&PointCloudFrameTrans::pointCloudCallback, this, std::placeholders::_1));
        
    }

private:
    // 特别标注一下，曾经对自己这段代码产生了质疑，一定不要忘记输入的点云是lio发布的！！
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        try
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*input_cloud, *cloud);

            // pcl::VoxelGrid<pcl::PointXYZI> filter;
            // filter.setInputCloud(cloud);
            // filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            // filter.filter(*cloud);
            // 进行坐标转换
            if (!tf_initialized_) {
                // try {
                //   geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                //     output_frame_, input_cloud->header.frame_id, rclcpp::Time(0), rclcpp::Duration(5, 0));
                //   tf2::Transform tf_base_frame_to_lidar;
                //   tf2::fromMsg(transform_stamped.transform, tf_base_frame_to_lidar);
                //   tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar;
                //   Eigen::Isometry3d eigen_transform_d = tf2::transformToEigen(transform_stamped);
                //   T_target_source = eigen_transform_d.cast<float>().matrix();
                //   tf_initialized_ = true;
                // } catch (tf2::TransformException & ex) {
                //   RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
                //   return;
                // }
                return;
            }
        pcl::transformPointCloud(*cloud, *cloud, T_target_source);
        float curr_x, curr_y, curr_z;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            curr_x = vehicle_x_;
            curr_y = vehicle_y_;
            curr_z = vehicle_z_;
        }
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(curr_x - limit_x_, curr_x + limit_x_);
        pass.filter(*cloud);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(curr_y - limit_y_, curr_y + limit_y_);
        pass.filter(*cloud);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(curr_z - limit_z_, curr_z + limit_z_);
        pass.filter(*cloud);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>());
        cloud_normal->reserve(cloud->size());

        for (const auto& p : *cloud) {
            pcl::PointXYZINormal np;
            np.x = p.x;
            np.y = p.y;
            np.z = p.z;
            np.intensity = p.intensity;

            np.normal_x = 0.0f;
            np.normal_y = 0.0f;
            np.normal_z = 1.0f;
            np.curvature = 0.0f;

            cloud_normal->push_back(np);
        }

        publishcloud(cloud_normal);

        // publishcloud(cloud);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud: %s", ex.what());
        }
    }
    void publishTfTransform(const tf2::Transform& transform, const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        // 1. 设置TF基础信息
        tf_msg.header.stamp = stamp;                
        tf_msg.header.frame_id = output_frame_;     
        tf_msg.child_frame_id = lidar_frame_;       

        // 2. 填充TF变换数据
        tf_msg.transform.translation.x = transform.getOrigin().x();
        tf_msg.transform.translation.y = transform.getOrigin().y();
        tf_msg.transform.translation.z = transform.getOrigin().z();
        tf_msg.transform.rotation = tf2::toMsg(transform.getRotation());

        // 3. 发布TF变换（ROS 2标准TF发布方式）
        tf_broadcaster_->sendTransform(tf_msg);
    }
    void publishcloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = output_frame_;
        pub_->publish(output_msg);
    }
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        if (!tf_initialized_) {
            try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame_, tf2::TimePointZero);
            tf2::Transform tf_base_frame_to_lidar;
            tf2::fromMsg(transform_stamped.transform, tf_base_frame_to_lidar);
            tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar;
            tf_lidar_to_base_ = tf_base_frame_to_lidar.inverse();
            Eigen::Isometry3d eigen_transform_d = tf2::transformToEigen(transform_stamped);
            T_target_source = eigen_transform_d.cast<float>().matrix();
            tf_initialized_ = true;
            } catch (tf2::TransformException & ex) {
              RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
              return;
            }
        }
        tf2::Transform tf_lidar_odom_to_lidar;
        tf2::fromMsg(odom_msg->pose.pose, tf_lidar_odom_to_lidar);
        tf2::Transform tf_baseodom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;
        // geometry_msgs::msg::TransformStamped transform_stamped;
        // transform_stamped = tf_buffer_->lookupTransform(
        //     odom_msg->header.frame_id,      // 目标坐标系
        //     output_frame_,                  // 
        //     odom_msg->header.stamp,         // 时间戳
        //     rclcpp::Duration(1, 0)          // 超时时间
        // );
        // tf2::Transform tf_original_odom;
        // tf2::fromMsg(odom_msg->pose.pose, tf_original_odom);
        

        // 将查找的TF变换转换为tf2::Transform
        // tf2::Transform tf_transform;
        // tf2::fromMsg(transform_stamped.transform, tf_transform);

        // // 组合变换：目标坐标系 = TF变换 * 原始里程计变换
        // tf2::Transform tf_transformed_odom = tf_transform * tf_original_odom;
        nav_msgs::msg::Odometry out;
        out.header.stamp = odom_msg->header.stamp;
        out.header.frame_id = output_frame_;
        out.child_frame_id = lidar_frame_;
      
        const auto & origin = tf_baseodom_to_lidar.getOrigin();
        out.pose.pose.position.x = origin.x();
        out.pose.pose.position.y = origin.y();
        out.pose.pose.position.z = origin.z();
        out.pose.pose.orientation = tf2::toMsg(tf_baseodom_to_lidar.getRotation());
      
        pub_odom_->publish(out);
        tf2::Transform tf_baseodom_to_base = tf_baseodom_to_lidar * tf_lidar_to_base_;

        nav_msgs::msg::Odometry out_base;
        out_base.header.stamp = odom_msg->header.stamp;
        out_base.header.frame_id = output_frame_;
        out_base.child_frame_id = base_frame_;

        const auto & base_origin = tf_baseodom_to_base.getOrigin();
        out_base.pose.pose.position.x = base_origin.x();
        out_base.pose.pose.position.y = base_origin.y();
        out_base.pose.pose.position.z = base_origin.z();
        out_base.pose.pose.orientation = tf2::toMsg(tf_baseodom_to_base.getRotation());
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            vehicle_x_ = base_origin.x();
            vehicle_y_ = base_origin.y();
            vehicle_z_ = base_origin.z();
        }

        const tf2::Matrix3x3 R_base_lidar = tf_base_to_lidar_.getBasis();
        const tf2::Vector3 p_base_lidar = tf_base_to_lidar_.getOrigin();
        
        tf2::Vector3 v_lidar(
            odom_msg->twist.twist.linear.x,
            odom_msg->twist.twist.linear.y,
            odom_msg->twist.twist.linear.z);
        
        tf2::Vector3 w_lidar(
            odom_msg->twist.twist.angular.x,
            odom_msg->twist.twist.angular.y,
            odom_msg->twist.twist.angular.z);
        
        tf2::Vector3 w_base = R_base_lidar * w_lidar;
        tf2::Vector3 v_base = R_base_lidar * v_lidar - w_base.cross(p_base_lidar);
        
        out_base.twist.twist.linear.x = v_base.x();
        out_base.twist.twist.linear.y = v_base.y();
        out_base.twist.twist.linear.z = v_base.z();
        
        out_base.twist.twist.angular.x = w_base.x();
        out_base.twist.twist.angular.y = w_base.y();
        out_base.twist.twist.angular.z = w_base.z();
        
        out_base.twist.covariance = odom_msg->twist.covariance;
        

        pub_base_odom_->publish(out_base);
        // publishTfTransform(tf_baseodom_to_lidar, odom_msg->header.stamp);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_;
    std::string odom_in_;
    std::string odom_out_;
    std::string base_frame_ ;
    std::string lidar_frame_;
    std::string odom_base_out_;
    bool tran_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_base_odom_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Eigen::Matrix4f T_target_source;
    float leaf_size_ = 0.04f; 

    bool tf_initialized_ = false;
    tf2::Transform tf_odom_to_lidar_odom_;
    tf2::Transform tf_base_to_lidar_;
    tf2::Transform tf_lidar_to_base_;
    float limit_x_;
    float limit_y_;
    float limit_z_;
    bool enable_voxel_;
    std::mutex odom_mutex_;
    float vehicle_x_ = 0.0;
    float vehicle_y_ = 0.0;
    float vehicle_z_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFrameTrans>());
    rclcpp::shutdown();
    return 0;
}