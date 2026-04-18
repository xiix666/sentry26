//
// Created by thesky on 25-4-2.
//
#include <chrono>
#include <vector>
#include <deque>

#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 简化类型别名
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZINormal = pcl::PointCloud<pcl::PointXYZINormal>;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, PointCloud2Msg>;

// 辅助函数：旋转矩阵转欧拉角（弧度）
void rotationMatrixToEulerAngles(const Eigen::Matrix3d& R, 
            double& roll, double& pitch, double& yaw) {
    double sin_pitch = -R(2, 0);
    sin_pitch = std::max(std::min(sin_pitch, 1.0), -1.0);
    pitch = std::asin(sin_pitch);
    double cos_pitch = std::cos(pitch);

    const double EPS = 1e-6;
    if (std::fabs(cos_pitch) > EPS) {
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw = std::atan2(R(1, 0), R(0, 0));
    } else {
        roll = 0.0;
        yaw = std::atan2(-R(0, 1), R(1, 1));
    }
}

// 辅助函数：弧度转角度
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// 辅助函数：角度转弧度
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 辅助函数：生成初始变换矩阵（固定值：x=0.0, y=-0.245, z=-0.245；roll=1.62°, pitch=0.0°, yaw=-0.0°）
Eigen::Matrix4d createFixedInitialTransform() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // 1. 固定初始变换参数（从命令行参数复制）
    const double roll_deg = 1.57;    // --roll 1.62
    const double pitch_deg = 0.0;    // --pitch 0.0
    const double yaw_deg = 0.0;     // --yaw -0.0
    const double tx = 0.0;           // --x 0.0
    const double ty = -0.245;        // --y -0.245
    const double tz = -0.245;        // --z -0.245

    // 2. 欧拉角（度）转弧度
    double roll_rad = deg2rad(roll_deg);
    double pitch_rad = deg2rad(pitch_deg);
    double yaw_rad = deg2rad(yaw_deg);

    // 3. 欧拉角转旋转矩阵（Z-Y-X顺序：yaw-pitch-roll）
    Eigen::AngleAxisd roll_angle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw_rad, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();

    // 4. 设置旋转和平移矩阵
    T.block<3,3>(0,0) = R;
    T(0,3) = tx;
    T(1,3) = ty;
    T(2,3) = tz;

    // 打印固定初始变换（方便验证）
    RCLCPP_WARN(rclcpp::get_logger("CalibLidar"), "=== Fixed Initial Transform ===");
    RCLCPP_WARN(rclcpp::get_logger("CalibLidar"), "Rotation (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                roll_deg, pitch_deg, yaw_deg);
    RCLCPP_WARN(rclcpp::get_logger("CalibLidar"), "Translation (m): X=%.3f, Y=%.3f, Z=%.3f",
                tx, ty, tz);
    RCLCPP_WARN(rclcpp::get_logger("CalibLidar"), "Initial matrix:\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf",
                T(0,0), T(0,1), T(0,2), T(0,3),
                T(1,0), T(1,1), T(1,2), T(1,3),
                T(2,0), T(2,1), T(2,2), T(2,3),
                T(3,0), T(3,1), T(3,2), T(3,3));

    return T;
}

class CalibLidar : public rclcpp::Node{
public:
    CalibLidar(std::string name):Node(name){
        RCLCPP_WARN(this->get_logger(), "CalibLidar start");

        T = createFixedInitialTransform();
        // 1. 初始化订阅器
        mid70_sub.subscribe(this, "/livox/lidar/pointcloud_192_168_2_120");
        avia_sub.subscribe(this, "/livox/lidar/pointcloud_192_168_2_121");

        // 2. 初始化时间同步器
        sync = std::make_unique<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(20),  
            avia_sub,              
            mid70_sub              
        );
        sync->setMaxIntervalDuration(rclcpp::Duration(0, 10000000)); 
        sync->registerCallback(&CalibLidar::PcTimeSynC, this);

        // 3. 基础参数配置（移除初始变换相关参数，改用固定值）
        this->declare_parameter<int>("accumulate_time", 5);
        this->declare_parameter<double>("icp_cost_thres", 0.1);
        this->declare_parameter<double>("voxel_size", 0.1);
        
        // 读取基础参数
        accumulate_time = this->get_parameter("accumulate_time").as_int();
        icp_cost_thres_ = this->get_parameter("icp_cost_thres").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();


        // 禁用手动选点（直接用固定初始值）
        use_manual_init_ = false;
        manual_aligned_ = true; // 标记初始变换已生成，无需手动选点
    }

    ~CalibLidar(){}

private:
    // 订阅器和同步器
    message_filters::Subscriber<PointCloud2Msg> mid70_sub;
    message_filters::Subscriber<PointCloud2Msg> avia_sub;
    std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

    // 配准参数
    int accumulate_time = 30;
    double icp_cost_thres_ = 0.3;
    double voxel_size_ = 0.2;
    bool manual_aligned_ = false;    // 手动配准完成标志（直接设为true）
    bool auto_aligned_ = false;      // 自动ICP完成标志
    bool use_manual_init_ = false;   // 禁用手动初始值
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); // 变换矩阵（初始+优化）
    int num = 0;
    // 累加点云
    std::vector<PointCloudXYZ::Ptr> accumulated_clouds_1; // Avia
    std::vector<PointCloudXYZ::Ptr> accumulated_clouds_2; // Mid70

    /**
     * @brief Open3D手动选点（保留但禁用，如需启用可改use_manual_init_为true）
     */
    std::vector<size_t> select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd){
        RCLCPP_INFO(this->get_logger(), "Select points...");
        RCLCPP_INFO(this->get_logger(), "  Use [shift + left click] to pick points.");
        RCLCPP_INFO(this->get_logger(), "  Use [shift + right click] to undo point picking.");
        RCLCPP_INFO(this->get_logger(), "  After picking points, press 'Q' to close the window.");

        open3d::visualization::VisualizerWithEditing vis_;
        vis_.CreateVisualizerWindow("Select Points", 1920, 1080);
        vis_.AddGeometry(pcd);
        vis_.Run();
        vis_.DestroyVisualizerWindow();
        return vis_.GetPickedPoints();
    }

    /**
     * @brief 手动配准（保留但禁用）
     */
    Eigen::Matrix4d manual_trans(std::shared_ptr<open3d::geometry::PointCloud> source, 
                                 std::shared_ptr<open3d::geometry::PointCloud> target){
        auto picked_source = select_points(source);
        auto picked_target = select_points(target);
        assert(picked_source.size() >= 3 && picked_target.size() >= 3);
        assert(picked_source.size() == picked_target.size());

        std::vector<Eigen::Vector2i> correspondences;
        for (size_t i = 0; i < picked_source.size(); ++i)
            correspondences.emplace_back(picked_source[i], picked_target[i]);

        open3d::pipelines::registration::TransformationEstimationPointToPoint pointToPoint;
        return pointToPoint.ComputeTransformation(*source, *target, correspondences);
    }

    /**
     * @brief 同步回调函数：处理Avia和Mid70点云
     */
    void PcTimeSynC(const PointCloud2Msg::SharedPtr msg1, const PointCloud2Msg::SharedPtr msg2) {
        // 1. 转换ROS点云到PCL点云
        PointCloudXYZ::Ptr avia_cloud(new PointCloudXYZ());
        PointCloudXYZ::Ptr mid70_cloud(new PointCloudXYZ());
        pcl::fromROSMsg(*msg1, *avia_cloud);
        pcl::fromROSMsg(*msg2, *mid70_cloud);

        // 2. 移除NaN点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*avia_cloud, *avia_cloud, indices);
        pcl::removeNaNFromPointCloud(*mid70_cloud, *mid70_cloud, indices);

        // 3. 点云累加
        if(accumulated_clouds_1.size() < accumulate_time){
            accumulated_clouds_1.push_back(avia_cloud);
            accumulated_clouds_2.push_back(mid70_cloud);
            return;
        } else {
            accumulated_clouds_1.erase(accumulated_clouds_1.begin());
            accumulated_clouds_1.push_back(avia_cloud);
            accumulated_clouds_2.erase(accumulated_clouds_2.begin());
            accumulated_clouds_2.push_back(mid70_cloud);
        }

        // 4. 合并累加点云并滤波
        PointCloudXYZ::Ptr target_cloud(new PointCloudXYZ()); // Avia（目标）
        PointCloudXYZ::Ptr source_cloud(new PointCloudXYZ()); // Mid70（源）

        // 4.1 合并Avia点云
        for(auto& acc_cloud : accumulated_clouds_1){
            for (const auto& point : *acc_cloud){
                if(point.x > -35 && point.x < 35 && 
                   point.y > -35 && point.y < 35 && 
                   point.z < 7.6){
                    target_cloud->push_back(point);
                }
            }
        }

        // 4.2 合并Mid70点云
        for(auto& acc_cloud : accumulated_clouds_2){
            for (const auto& point : *acc_cloud){
                if(point.x > -35 && point.x < 35 && 
                   point.y > -35 && point.y < 35 && 
                   point.z < 7.6){
                    source_cloud->push_back(point);
                }
            }
        }

        // 5. 跳过手动配准（直接用固定初始值）
        if(use_manual_init_ && !manual_aligned_){
            auto avia_o3d = std::make_shared<open3d::geometry::PointCloud>();
            auto mid70_o3d = std::make_shared<open3d::geometry::PointCloud>();

            for(const auto& point : target_cloud->points){
                avia_o3d->points_.emplace_back(point.x, point.y, point.z);
            }
            for(const auto& point : source_cloud->points){
                mid70_o3d->points_.emplace_back(point.x, point.y, point.z);
            }

            T = manual_trans(mid70_o3d, avia_o3d);
            manual_aligned_ = true;
            RCLCPP_WARN(this->get_logger(), "Manual init transform done!");
            
            double roll, pitch, yaw;
            rotationMatrixToEulerAngles(T.block<3,3>(0,0), roll, pitch, yaw);
            RCLCPP_WARN(this->get_logger(), "Manual init: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°",
                        rad2deg(roll), rad2deg(pitch), rad2deg(yaw));
            RCLCPP_WARN(this->get_logger(), "Manual init: Tx=%.2f, Ty=%.2f, Tz=%.2f",
                        T(0,3), T(1,3), T(2,3));
        }

        // 6. 自动ICP细标定（基于固定初始变换）
        if(!auto_aligned_){
            // 6.1 降采样
            PointCloudXYZ::Ptr downsampled_target = std::make_shared<PointCloudXYZ>(); 
            PointCloudXYZ::Ptr downsampled_source = std::make_shared<PointCloudXYZ>(); 

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
            voxelgrid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxelgrid.setInputCloud(target_cloud);
            voxelgrid.filter(*downsampled_target);
            voxelgrid.setInputCloud(source_cloud);
            voxelgrid.filter(*downsampled_source);

            // 6.2 应用固定初始变换到源点云
            PointCloudXYZ::Ptr source_transformed = std::make_shared<PointCloudXYZ>();
            pcl::transformPointCloud(*downsampled_source, *source_transformed, T.cast<float>());

            // 6.3 计算法向量
            PointCloudXYZINormal::Ptr sourceCloudNormal = std::make_shared<PointCloudXYZINormal>();
            PointCloudXYZINormal::Ptr targetCloudNormal = std::make_shared<PointCloudXYZINormal>();
            pcl::copyPointCloud(*source_transformed, *sourceCloudNormal);
            pcl::copyPointCloud(*downsampled_target, *targetCloudNormal);

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::PointCloud<pcl::Normal>::Ptr source_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
            pcl::PointCloud<pcl::Normal>::Ptr target_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
            
            auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
            auto tree2 = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

            ne.setInputCloud(source_transformed); 
            ne.setSearchMethod(tree);
            ne.setKSearch(30);
            ne.compute(*source_normals);
            pcl::copyPointCloud(*source_normals, *sourceCloudNormal); 

            ne.setInputCloud(downsampled_target); 
            ne.setSearchMethod(tree2);
            ne.setKSearch(30);
            ne.compute(*target_normals);
            pcl::copyPointCloud(*target_normals, *targetCloudNormal);

            // 6.4 GICP细标定
            auto gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal>>();
            pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_1 = std::make_shared<pcl::search::KdTree<pcl::PointXYZINormal>>();
            pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_2 = std::make_shared<pcl::search::KdTree<pcl::PointXYZINormal>>();
            
            gicp->setSearchMethodSource(tree_1);
            gicp->setSearchMethodTarget(tree_2);
            gicp->setInputTarget(targetCloudNormal);
            gicp->setInputSource(sourceCloudNormal);
            gicp->setMaximumIterations(100);        
            gicp->setMaxCorrespondenceDistance(0.5); 
            gicp->setTransformationEpsilon(1e-8);   
            gicp->setRotationEpsilon(1e-8);         
            gicp->setEuclideanFitnessEpsilon(1e-6);
            gicp->setUseReciprocalCorrespondences(true);

            // 6.5 执行ICP配准
            PointCloudXYZINormal::Ptr aligned(new PointCloudXYZINormal());
            gicp->align(*aligned);

            // 6.6 输出配准结果
            double fitness_score = gicp->getFitnessScore();
            RCLCPP_INFO(this->get_logger(), "ICP iteration %d, fitness score: %.6f (thres=%.6f)",
                        num+1, fitness_score, icp_cost_thres_);
            std::cout << gicp->getFinalTransformation() << std::endl;

            // 6.7 更新变换矩阵
            if (gicp->hasConverged()) {
                Eigen::Matrix4f icp_delta = gicp->getFinalTransformation();
                T = (icp_delta.cast<double>() * T).eval();
                num ++ ;

                // 6.8 判断配准成功
                if(fitness_score < icp_cost_thres_){
                    auto_aligned_ = true;
                    double roll, pitch, yaw;
                    rotationMatrixToEulerAngles(T.block<3,3>(0,0), roll, pitch, yaw);
                    RCLCPP_WARN(this->get_logger(), "=== Final Calibration Result ===");
                    RCLCPP_WARN(this->get_logger(), "Fitness score: %.6f", fitness_score);
                    RCLCPP_WARN(this->get_logger(), "Rotation (deg): Roll=%.4f, Pitch=%.4f, Yaw=%.4f",
                                rad2deg(roll), rad2deg(pitch), rad2deg(yaw));
                    RCLCPP_WARN(this->get_logger(), "Translation (m): Tx=%.4f, Ty=%.4f, Tz=%.4f",
                                T(0,3), T(1,3), T(2,3));
                    RCLCPP_WARN(this->get_logger(), "Final transform matrix:\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf",
                                T(0,0), T(0,1), T(0,2), T(0,3),
                                T(1,0), T(1,1), T(1,2), T(1,3),
                                T(2,0), T(2,1), T(2,2), T(2,3),
                                T(3,0), T(3,1), T(3,2), T(3,3));
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "ICP did not converge! Reset accumulate clouds.");
                accumulated_clouds_1.clear();
                accumulated_clouds_2.clear();
            }
        }

        // 7. 打印最终结果
        if(auto_aligned_){
            RCLCPP_WARN(this->get_logger(), "=== Calibration Done! Final Matrix ===");
            std::cout << T << std::endl;
            std::cout << "Total ICP iterations: " << num << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibLidar>("CalibLidar");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
