

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

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZINormal = pcl::PointCloud<pcl::PointXYZINormal>;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, PointCloud2Msg>;

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

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

Eigen::Matrix4d createFixedInitialTransform() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    const double roll_deg = 1.57;
    const double pitch_deg = 0.0;
    const double yaw_deg = 0.0;
    const double tx = 0.0;
    const double ty = -0.245;
    const double tz = -0.245;

    double roll_rad = deg2rad(roll_deg);
    double pitch_rad = deg2rad(pitch_deg);
    double yaw_rad = deg2rad(yaw_deg);

    Eigen::AngleAxisd roll_angle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw_rad, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();

    T.block<3,3>(0,0) = R;
    T(0,3) = tx;
    T(1,3) = ty;
    T(2,3) = tz;

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
        mid70_sub.subscribe(this, "/livox/lidar/pointcloud_192_168_2_120");
        avia_sub.subscribe(this, "/livox/lidar/pointcloud_192_168_2_121");

        sync = std::make_unique<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(20),  
            avia_sub,              
            mid70_sub              
        );
        sync->setMaxIntervalDuration(rclcpp::Duration(0, 10000000)); 
        sync->registerCallback(&CalibLidar::PcTimeSynC, this);

        this->declare_parameter<int>("accumulate_time", 5);
        this->declare_parameter<double>("icp_cost_thres", 0.1);
        this->declare_parameter<double>("voxel_size", 0.1);
        
        accumulate_time = this->get_parameter("accumulate_time").as_int();
        icp_cost_thres_ = this->get_parameter("icp_cost_thres").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();

        use_manual_init_ = false;
        manual_aligned_ = true;
    }

    ~CalibLidar(){}

private:
    message_filters::Subscriber<PointCloud2Msg> mid70_sub;
    message_filters::Subscriber<PointCloud2Msg> avia_sub;
    std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

    int accumulate_time = 30;
    double icp_cost_thres_ = 0.3;
    double voxel_size_ = 0.2;
    bool manual_aligned_ = false;
    bool auto_aligned_ = false;
    bool use_manual_init_ = false;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int num = 0;

    std::vector<PointCloudXYZ::Ptr> accumulated_clouds_1;
    std::vector<PointCloudXYZ::Ptr> accumulated_clouds_2;

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

    void PcTimeSynC(const PointCloud2Msg::SharedPtr msg1, const PointCloud2Msg::SharedPtr msg2) {

        PointCloudXYZ::Ptr avia_cloud(new PointCloudXYZ());
        PointCloudXYZ::Ptr mid70_cloud(new PointCloudXYZ());
        pcl::fromROSMsg(*msg1, *avia_cloud);
        pcl::fromROSMsg(*msg2, *mid70_cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*avia_cloud, *avia_cloud, indices);
        pcl::removeNaNFromPointCloud(*mid70_cloud, *mid70_cloud, indices);

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

        PointCloudXYZ::Ptr target_cloud(new PointCloudXYZ());
        PointCloudXYZ::Ptr source_cloud(new PointCloudXYZ());

        for(auto& acc_cloud : accumulated_clouds_1){
            for (const auto& point : *acc_cloud){
                if(point.x > -35 && point.x < 35 && 
                   point.y > -35 && point.y < 35 && 
                   point.z < 7.6){
                    target_cloud->push_back(point);
                }
            }
        }

        for(auto& acc_cloud : accumulated_clouds_2){
            for (const auto& point : *acc_cloud){
                if(point.x > -35 && point.x < 35 && 
                   point.y > -35 && point.y < 35 && 
                   point.z < 7.6){
                    source_cloud->push_back(point);
                }
            }
        }

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

        if(!auto_aligned_){

            PointCloudXYZ::Ptr downsampled_target = std::make_shared<PointCloudXYZ>(); 
            PointCloudXYZ::Ptr downsampled_source = std::make_shared<PointCloudXYZ>(); 

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
            voxelgrid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxelgrid.setInputCloud(target_cloud);
            voxelgrid.filter(*downsampled_target);
            voxelgrid.setInputCloud(source_cloud);
            voxelgrid.filter(*downsampled_source);

            PointCloudXYZ::Ptr source_transformed = std::make_shared<PointCloudXYZ>();
            pcl::transformPointCloud(*downsampled_source, *source_transformed, T.cast<float>());

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

            PointCloudXYZINormal::Ptr aligned(new PointCloudXYZINormal());
            gicp->align(*aligned);

            double fitness_score = gicp->getFitnessScore();
            RCLCPP_INFO(this->get_logger(), "ICP iteration %d, fitness score: %.6f (thres=%.6f)",
                        num+1, fitness_score, icp_cost_thres_);
            std::cout << gicp->getFinalTransformation() << std::endl;

            if (gicp->hasConverged()) {
                Eigen::Matrix4f icp_delta = gicp->getFinalTransformation();
                T = (icp_delta.cast<double>() * T).eval();
                num ++ ;

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
