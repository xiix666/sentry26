// #include <so3_math.h>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "li_initialization.h"

using namespace std;
namespace fs = std::filesystem;
#define PUBFRAME_PERIOD (20)

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br;

bool much_point = false;
bool read_pose = false;
std::atomic<bool> prior_map_loaded = false;
std::thread prior_map_thread;
const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;
// std::mutex tf_file_mutex_;                  // 文件写入互斥锁（线程安全）
// std::chrono::steady_clock::time_point last_tf_write_time_; // 上次写入时间
// std::ofstream tf_txt_file_(pose_txt, std::ios::out | std::ios::app);                 // TXT文件流
// const double TF_WRITE_INTERVAL = 0.1;       // 写入间隔（0.1秒）
//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;//表面点云下采样
pcl::VoxelGrid<PointType> downSizeFilterMap;//地图点云下采样

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

int sleep_time = 0;

auto LOGGER = rclcpp::get_logger("laserMapping");

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

tf2::Transform getStaticTF(const std::string& parent_frame, const std::string& child_frame) {
  tf2::Transform static_tf;
  try {
      geometry_msgs::msg::TransformStamped tf_msg = 
          tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero, std::chrono::seconds(1));
      tf2::fromMsg(tf_msg.transform, static_tf);
  } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("static_tf_reader"), "读取静态TF %s→%s失败: %s",
                   parent_frame.c_str(), child_frame.c_str(), ex.what());
      static_tf.setIdentity(); 
  }
  return static_tf;
}
void checkTFRecordFile(const std::string& filename = "pose.txt") {

  // std::lock_guard<std::mutex> lock(tf_file_mutex_);

  std::ifstream file(filename);
  if (!file.is_open()) {
      std::cerr << "Error: 无法打开TF记录文件 " << filename << std::endl;
      read_pose = false;
      return;
  }

  std::string line;
  bool has_non_zero_value = false; 
  const double EPS = 1e-6;         

  while (std::getline(file, line) && !has_non_zero_value) {
      // 跳过空行
      if (line.empty()) continue;

      // 3. 解析单行数据（格式：Translation: x,y,z,Rotation: x,y,z,w）
      // 步骤1：提取Translation部分（x,y,z）
      size_t trans_pos = line.find("Translation: ");
      size_t rot_pos = line.find("Rotation: ");
      if (trans_pos == std::string::npos || rot_pos == std::string::npos) {
          continue; // 格式错误，跳过该行
      }

      // 步骤2：截取Translation的数值部分（x,y,z）
      std::string trans_str = line.substr(trans_pos + 13, rot_pos - (trans_pos + 13) - 1);
      // 步骤3：截取Rotation的数值部分（x,y,z,w）
      std::string rot_str = line.substr(rot_pos + 9);

      // 4. 解析Translation数值（x,y,z）
      std::vector<double> trans_values;
      std::stringstream trans_ss(trans_str);
      std::string token;
      while (std::getline(trans_ss, token, ',')) {
          try {
              trans_values.push_back(std::stod(token));
          } catch (...) {
              break; 
          }
      }

      // 5. 解析Rotation数值（x,y,z,w）
      std::vector<double> rot_values;
      std::stringstream rot_ss(rot_str);
      while (std::getline(rot_ss, token, ',')) {
          try {
              rot_values.push_back(std::stod(token));
          } catch (...) {
              break; 
          }
      }

      if (trans_values.size() == 3) {
          for (double val : trans_values) {
              if (std::fabs(val) > EPS) { 
                  has_non_zero_value = true;
                  break;
              }
          }
      }

      if (!has_non_zero_value && rot_values.size() == 4) {
          for (double val : rot_values) {
              if (std::fabs(val) > EPS) {
                  has_non_zero_value = true;
                  break;
              }
          }
      }
      tf2::Transform odom_init2lidar;
      odom_init2lidar.setOrigin(tf2::Vector3(trans_values[0], trans_values[1], trans_values[2]));
      odom_init2lidar.setRotation(tf2::Quaternion(rot_values[0], rot_values[1], rot_values[2], rot_values[3]));
      tf2::Transform odom2odom_init = getStaticTF("odom", "odom_init"); 
      tf2::Transform lidar2base = getStaticTF("lidar_link", "base_link");         
      tf2::Transform odom_init2base = odom_init2lidar * lidar2base;
      tf2::Transform odom2base = odom2odom_init * odom_init2base;
      tf2::Vector3 flat_trans = odom2base.getOrigin();
      tf2::Matrix3x3 flat_rot(odom2base.getRotation());
      double roll, pitch, yaw;
      flat_rot.getRPY(roll, pitch, yaw);

      geometry_msgs::msg::TransformStamped odom2base_flat;
      odom2base_flat.header.frame_id = "map";
      odom2base_flat.child_frame_id = "odom";
      // 替换为你的实际时间戳（和原始变换一致）
      odom2base_flat.header.stamp = get_ros_time(lidar_end_time); 
      odom2base_flat.transform.translation.x = flat_trans.x();
      odom2base_flat.transform.translation.y = flat_trans.y();
      odom2base_flat.transform.translation.z = flat_trans.z();
      tf2::Quaternion flat_q = odom2base.getRotation();
      odom2base_flat.transform.rotation.x = flat_q.x();
      odom2base_flat.transform.rotation.y = flat_q.y();
      odom2base_flat.transform.rotation.z = flat_q.z();
      odom2base_flat.transform.rotation.w = flat_q.w();

      tf_br->sendTransform(odom2base_flat);
  }

  // 7. 关闭文件，更新read_pose
  file.close();
  read_pose = has_non_zero_value;

}
PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string & file_path)
{
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(LOGGER, "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(LOGGER, "Loaded %zu points from %s", pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

inline void dump_lio_state_to_log(FILE * fp)
{
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // omega
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // Acc
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));  // Bias_g
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}
// void pointBodyTolidar_world(PointType const * const pi, PointType * const po)
// {
//   V3D p_body(pi->x, pi->y, pi->z);
//   V3D p_global;
//   if (extrinsic_est_en) { 
//     if (!use_imu_as_input) {
//       p_global =
//         kf_output.x_.rot * p_body +
//         kf_output.x_.pos;
//     } else {
//       p_global = kf_input.x_.rot * p_body  +
//                  kf_input.x_.pos;
//     }
//   } else {
//     if (!use_imu_as_input) {
//       p_global = kf_output.x_.rot  * p_body +
//                  kf_output.x_.pos;  // .normalized()
//     } else {
//       p_global = kf_input.x_.rot *  p_body  +
//                  kf_input.x_.pos;  // .normalized()
//     }
//   }
//   po->x = p_global(0);
//   po->y = p_global(1);
//   po->z = p_global(2);
//   po->intensity = pi->intensity;
// }
  tf2::Transform getOdomInitToCameraInit()
  {
    static bool initialized = false;
    static bool got_reloc_tf = false;
    static bool warned_once = false;
    static tf2::Transform T_odom_init_camera_init;
  
    if (!initialized) {
      T_odom_init_camera_init.setIdentity();
      initialized = true;
    }
  
    // 如果已经成功读到一次，就一直用这个变换
    if (got_reloc_tf) {
      return T_odom_init_camera_init;
    }
  
    try {
      auto tf_msg = tf_buffer_->lookupTransform(
        "odom_init",
        "camera_init",
        tf2::TimePointZero);
  
      tf2::fromMsg(tf_msg.transform, T_odom_init_camera_init);
      got_reloc_tf = true;
  
      const auto t = T_odom_init_camera_init.getOrigin();
      const auto q = T_odom_init_camera_init.getRotation();
  
      RCLCPP_INFO(
        LOGGER,
        "Got relocalization TF odom_init -> camera_init: "
        "t=[%.3f, %.3f, %.3f], q=[%.6f, %.6f, %.6f, %.6f]",
        t.x(), t.y(), t.z(),
        q.x(), q.y(), q.z(), q.w());
  
    } catch (const tf2::TransformException & ex) {
      if (!warned_once) {
        RCLCPP_WARN(
          LOGGER,
          "No TF odom_init -> camera_init yet, use identity temporarily: %s",
          ex.what());
        warned_once = true;
      }
  
      // 没读到就保持单位变换
      T_odom_init_camera_init.setIdentity();
    }
  
    return T_odom_init_camera_init;
  }
void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) *
        filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = init_frame;
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_lidar(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_body, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar_link";
    pubLaserCloudFullRes->publish(laserCloudmsg);

  }
}

void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {

    PointCloudXYZI::Ptr cloud_odom_init(new PointCloudXYZI());
    cloud_odom_init->resize(feats_down_world->size());

    tf2::Transform T_odom_init_camera_init = getOdomInitToCameraInit();

    for (size_t i = 0; i < feats_down_world->size(); ++i) {
      const auto& p = feats_down_world->points[i];

      tf2::Vector3 pc(p.x, p.y, p.z);
      tf2::Vector3 po = T_odom_init_camera_init * pc;

      auto& q = cloud_odom_init->points[i];
      q.x = po.x();
      q.y = po.y();
      q.z = po.z();
      q.intensity = p.intensity;
    }
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*cloud_odom_init, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudmsg.header.frame_id = "lidar_odom";
    laserCloudmsg.header.frame_id = init_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg); //imu初始位姿为原点

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      if(pcl_wait_save->size() > 100000){
        much_point = true;
      }
      if(!much_point){
        *pcl_wait_save += *feats_down_world;
      }

      static int scan_wait_num = 0;
      static int saved_this_run = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {

      double px, py, pz;
      V3D euler_rpy;

      if (!use_imu_as_input) {
        px = kf_output.x_.pos(0);
        py = kf_output.x_.pos(1);
        pz = kf_output.x_.pos(2);
        euler_rpy = SO3ToEuler(kf_output.x_.rot);
      } else {
        px = kf_input.x_.pos(0);
        py = kf_input.x_.pos(1);
        pz = kf_input.x_.pos(2);
        euler_rpy = SO3ToEuler(kf_input.x_.rot);
      }

      double roll  = euler_rpy(0);
      double pitch = euler_rpy(1);
      double yaw   = euler_rpy(2);

      char pose_str[128];
      snprintf(
        pose_str,
        sizeof(pose_str),
        "%.2f-%.2f-%.2f-%.2f-%.2f-%.2f",
        px, py, pz, roll, pitch, yaw);

      fs::path all_points_path;

      // 找到下一个不存在的 pcd 文件，避免覆盖
      do {
        pcd_index++;

        if (saved_this_run >= 1000) {
          RCLCPP_WARN(
            rclcpp::get_logger("pcd_save"),
            "This run has already saved 1000 PCD files, stop saving more.");
          return;
        }

        all_points_path = fs::path(save_path) /
          (std::to_string(pcd_index) + "_" + std::string(pose_str) + ".pcd");

      } while (fs::exists(all_points_path));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_path.string() << '\n';
        pcd_writer.writeBinary(all_points_path.string(), *pcl_wait_save);
        saved_this_run++;
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T & out)
{
  // Static variable, initialized to true, only effective on the first call
  static bool is_first_kf = true;

  auto set_output_from_kf = [&](const auto & kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    if (enable_prior_pcd && is_first_kf) {
      // Execute only on the first call
      kf_output.x_.pos(0) = init_pose[0];
      kf_output.x_.pos(1) = init_pose[1];
      kf_output.x_.pos(2) = init_pose[2];
      set_output_from_kf(kf_output);
      is_first_kf = false;  // Set is_first_kf to false after the first call
    } else {
      set_output_from_kf(kf_output);
    }
  } else {
    set_output_from_kf(kf_input);
  }
}

void publish_odometry(const rclcpp::Publisher<
  nav_msgs::msg::Odometry>::SharedPtr& pubOdomAftMapped,
  std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_br) {
      odomAftMapped.header.frame_id = init_frame;
      odomAftMapped.child_frame_id = "body";
    if (publish_odometry_without_downsample) {
      odomAftMapped.header.stamp = get_ros_time(time_current);
    } else {
      odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    // set_posestamp(odomAftMapped.pose.pose);
    geometry_msgs::msg::Pose pose_camera_init_body;
    set_posestamp(pose_camera_init_body);

    tf2::Transform T_camera_init_body;
    tf2::fromMsg(pose_camera_init_body, T_camera_init_body);

    tf2::Transform T_odom_init_camera_init = getOdomInitToCameraInit();
    tf2::Transform T_odom_init_body =
    T_odom_init_camera_init * T_camera_init_body;
    // 
    const tf2::Vector3 trans = T_odom_init_body.getOrigin();
    tf2::Quaternion q = T_odom_init_body.getRotation();
    q.normalize();

    odomAftMapped.pose.pose.position.x = trans.x();
    odomAftMapped.pose.pose.position.y = trans.y();
    odomAftMapped.pose.pose.position.z = trans.z();

    odomAftMapped.pose.pose.orientation.x = q.x();
    odomAftMapped.pose.pose.orientation.y = q.y();
    odomAftMapped.pose.pose.orientation.z = q.z();
    odomAftMapped.pose.pose.orientation.w = q.w();
    Eigen::Vector3d vel_world;
    Eigen::Vector3d omg_body;

    if (!use_imu_as_input) {
    vel_world = kf_output.x_.vel;
    omg_body = kf_output.x_.omg;
    } else {
    vel_world = kf_input.x_.vel;

    // use_imu_as_input 分支如果没有 x_.omg，就先置零
    omg_body.setZero();
    }

    // 当前姿态：body/lidar 到 world 的旋转
    Eigen::Matrix3d R_world_body;
    if (!use_imu_as_input) {
    R_world_body = kf_output.x_.rot;
    } else {
    R_world_body = kf_input.x_.rot;
    }

// 世界系线速度转到 child_frame_id 系
    Eigen::Vector3d vel_body = R_world_body.transpose() * vel_world;

    odomAftMapped.twist.twist.linear.x = vel_body.x();
    odomAftMapped.twist.twist.linear.y = vel_body.y();
    odomAftMapped.twist.twist.linear.z = vel_body.z();

    odomAftMapped.twist.twist.angular.x = omg_body.x();
    odomAftMapped.twist.twist.angular.y = omg_body.y();
    odomAftMapped.twist.twist.angular.z = omg_body.z();
    pubOdomAftMapped->publish(odomAftMapped);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = init_frame;
    transform.child_frame_id = lidar_frame;
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);
    // if(write_pose_txt){
    // std::lock_guard<std::mutex> lock(tf_file_mutex_); 
    // auto now = std::chrono::steady_clock::now();
    // double elapsed_seconds = std::chrono::duration<double>(now - last_tf_write_time_).count();
    // if (elapsed_seconds >= TF_WRITE_INTERVAL && tf_txt_file_.is_open()) {
    //     // 2. 格式化写入数据（时间戳+平移+旋转，逗号分隔，便于后续解析）
    //     tf_txt_file_ << std::fixed << std::setprecision(6); // 保留6位小数
    //     // 写入平移（x,y,z）
    //     tf_txt_file_ << "Translation: " << transform.transform.translation.x << "," 
    //                  << transform.transform.translation.y << "," 
    //                  << transform.transform.translation.z << ",";
    //     // 写入旋转（x,y,z,w）
    //     tf_txt_file_ << "Rotation: " << transform.transform.rotation.x << "," 
    //                  << transform.transform.rotation.y << "," 
    //                  << transform.transform.rotation.z << "," 
    //                  << transform.transform.rotation.w << std::endl;
        
    //     // 3. 刷新文件缓冲区（确保数据立即写入，避免缓存丢失）
    //     tf_txt_file_.flush();
    //     // 4. 更新上次写入时间
    //     last_tf_write_time_ = now;
    // }
  // }
  }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = "odom";
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

int main(int argc, char ** argv)
{
  //创建节点
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");
  //多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
  
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  std::cout << "lidar_type: " << lidar_type << '\n';
  ivox_ = std::make_shared<IVoxType>(ivox_options_);
  //初始要发布的路径消息
  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = "odom";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
         aver_time_solve = 0, aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  //确定体速滤波的体素大小
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
  //用流给雷达原点相对imu坐标系的变换赋初值
  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) { 
      // 将外参赋给输出滤波器kf_output的状态
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else { // 如果使用IMU作为KF输入（紧耦合模式）
      // 将外参赋给输入滤波器kf_input的状态
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }
  //给imu预处理模块传递参数
  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;
  //多传感器融合状态估计系统（如LIO）初始化
  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);//f(x) f(x)雅可比函数 h(x)
  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  //初始化滤波器协方差矩阵
  Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  //设置过程噪声协方差
  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
  /*** debug record ***/
  // FILE * fp;
  // string pos_log_dir = root_dir + "/Log/pos_log.txt";
  // fp = fopen(pos_log_dir.c_str(), "w");
  // open_file();
  // last_tf_write_time_ = std::chrono::steady_clock::now();
  // // tf_txt_file_.open(pose_txt, std::ios::out | std::ios::app);
  // if (!tf_txt_file_.is_open()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("Lasermapping"), "Failed to pose.txt!");
  // }
  // checkTFRecordFile(pose_txt); 
  /*** ROS subscribe initialization ***/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
  } else {
    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
  }
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(5));
  odom_qos.reliable();
  odom_qos.durability_volatile();
  auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
  auto pub_laser_cloud_full_res =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered",
                                                          odom_qos);
  auto pub_laser_cloud_full_res_body =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>(
          "cloud_registered_body", 20);
  auto pub_laser_cloud_effect =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 20);
  auto pub_laser_cloud_map =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
  auto pub_odom_aft_mapped =
      nh->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", odom_qos);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 20);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
  // checkTFRecordFile(pose_txt, tf_broadcaster); 
  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  //500hz循环频率
  rclcpp::Rate rate(500);

  while (rclcpp::ok()) {
    if (flg_exit) break;
    executor.spin_some();
    if (sync_packages(Measures)) {//数据同步 取出缓存数据到mea中
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        //重置imu处理器 清空点云 重置滤波器状态 标志位
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;//记录第一帧雷达点云的时间戳
        flg_first_scan = false;       
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);//记录第一帧imu的时间戳
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          //重力向量赋值给滤波器状态
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;
          //清空imu队列中时间戳早于第一帧雷达点云的imu数据
          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;//imu初始化完成
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);//重力加速度（重力向量模长）
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /*** downsample the feature points in a scan ***/
      t1 = omp_get_wtime();

      p_imu->Process(Measures, feats_undistort);
      //下采样
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);//对点云按时间戳排序
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      // {
      //   time_seq = time_compressing<int>(feats_down_body);
      //   feats_down_size = feats_down_body->points.size();//下采样后点云数量
      // }

      std::vector<double> bin_t_ref_sec;  
      if (use_batch)
      {
        time_seq = build_time_seq_by_ms_bin(feats_down_body, 0.02 /*1ms*/, &bin_t_ref_sec);
        // std::cout << "new time seq: " << time_seq.size() << std::endl;
        int sum = 0, mx = 0;
        for (auto c : time_seq)
        {
          sum += c;
          mx = std::max(mx, c);
        }
        // RCLCPP_INFO(LOGGER, "bins=%zu sum=%d max_bin_pts=%d",
        //             time_seq.size(), sum, mx);
        feats_down_size = feats_down_body->points.size();
      }
      else
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      }


      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;//归一化的平均加速度向量(重力方向)*重力标量
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);//计算初始旋转矩阵(temp_g->g)
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {//机体坐标系下的点云转换到世界坐标系
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        //初始化地图
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }
        if (enable_prior_pcd && !prior_map_loaded) {
          prior_map_thread = std::thread([&]() {
            // 异步加载+预处理先验点云
            auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            pcl::VoxelGrid<PointType> vg;
            vg.setInputCloud(map_cloud);
            vg.setLeafSize(0.5f, 0.5f, 0.5f);
            pcl::PointCloud<PointType>::Ptr map_cloud_down(new pcl::PointCloud<PointType>());
            vg.filter(*map_cloud_down);
            ivox_->AddPoints(map_cloud_down->points);
            prior_map_loaded = true;
          });
          prior_map_thread.detach(); // 分离线程，不阻塞主线程
        }
        if (init_feats_world->size() >= init_map_size) {
          if (enable_prior_pcd) {
            //加载先验地图点云
            // auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            // ivox_->AddPoints(map_cloud->points);
          } else {
            //在线构建地图
            ivox_->AddPoints(init_feats_world->points);
          }
          //发布初始地图
          publish_init_map(pub_laser_cloud_map);
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
       normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      crossmat_list.reserve(feats_down_size);
      pbody_list.reserve(feats_down_size);
      // pbody_ext_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z); //点云数据转换为三维向量 eigen格式
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU; //雷达坐标系转换到imu坐标系
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this); //点的反对称矩阵
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) { //不用imu作为输入
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/

        const int least_valid_point_num = 30;
        bool lidar_valid = (feats_down_size >= least_valid_point_num);

        if (lidar_valid) { //有雷达点云
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            if (use_batch)
            {
              time_current = pcl_beg_time + bin_t_ref_sec[k] / 1000.0; // 传播/更新到的绝对时间，现在你认为的状态x对应的点
            }
            else
            {
              PointType &point_body = feats_down_body->points[idx + time_seq[k]];
              time_current = point_body.curvature /1000.0 + pcl_beg_time; // （s）
            }
            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) { //弹出imu时间戳早于雷达的
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;       //用初始imu数据给角速度加速度
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);   //检查是否是最后一帧
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {  //弹出所有早于上一次预测的数据
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              //时间戳早于当前点
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;//与上一次预测的时间差
                kf_output.predict(dt, Q_output, input_in, true, false);// 状态预测 
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;//与上一次更新的时间差

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);// 协方差预测

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();//更新     imu观测修正
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {   //不按imu频率预测，用雷达时间更新协方差
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);//
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) { //雷达点云数量不足，跳过
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {     //
              idx = idx + time_seq[k];
              continue;
            }
            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/
              //里程计发布
              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {      
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "              //写入到imu_pbp.txt
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose()
                         << " " << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " " << kf_output.x_.bg.transpose()
                         << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }
            //坐标转换
            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?要预测的帧的时间戳在time_current和雷达开始时间+雷达时间窗口
              if (is_first_frame) {  //过滤掉早于雷达时间窗口的imu数据
                // {
                //   {
                //     while (get_time_sec(imu_next.header.stamp) <
                //            Measures.lidar_beg_time + lidar_time_inte) {
                //       // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                //       imu_deque.pop_front();
                //       if (imu_deque.empty()) break;
                //       imu_last = imu_next;
                //       imu_next = *(imu_deque.front());
                //     }
                //   }
                //   break;
                // }
                time_current = get_time_sec(imu_next.header.stamp);
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                // double dt = time_current - time_predict_last_const;  //当前imu帧与上一次预测的dt
                // {
                //   double dt_cov = time_current - time_update_last; //与上一次更新的时间差
                //   if (dt_cov > 0.0) {
                //     kf_output.predict(dt_cov, Q_output, input_in, false, true); // 状态预测
                //     time_update_last = time_current;
                //   }
                //   kf_output.predict(dt, Q_output, input_in, true, false); // 协方差预测
                // }

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                time_current = get_time_sec(imu_next.header.stamp);
                time_update_last = time_current;
                time_predict_last_const = time_current;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                // kf_output.update_iterated_dyn_share_IMU(); // 更新
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else { //
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            if (use_batch)
            {
              time_current = pcl_beg_time + bin_t_ref_sec[k] / 1000.0; // 传播/更新到的绝对时间，现在你认为的状态x对应的点
            }
            else
            {
              PointType &point_body = feats_down_body->points[idx + time_seq[k]];
              time_current = point_body.curvature /1000.0 + pcl_beg_time; // （s）
            }
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  //time_current;
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty()) break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            if (!kf_input.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }

            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose()
                         << " " << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  //time_current;
                }
                // kf_input.predict(dt, Q_input, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // M3D rot_cur_lidar;
      // {
      //     rot_cur_lidar = state.rot_end;
      // }
      // euler_cur = RotMtoEuler(rot_cur_lidar);
      // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      //                     (euler_cur(0), euler_cur(1), euler_cur(2));
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
      }

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      if (feats_down_size > 4) {
        if (enable_prior_pcd) {
          sleep_time++;
          if (sleep_time > 200) {
            MapIncremental();
          }
        } else {
          MapIncremental();
        }
      }
      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (path_en) publish_path(pub_path);
      if (scan_pub_en || pcd_save_en) publish_frame_world(pub_laser_cloud_full_res);
      // if (scan_pub_en) publish_frame_lidar(pub_laser_cloud_full_res);
      if (scan_pub_en && scan_body_pub_en) publish_frame_body(pub_laser_cloud_full_res_body);

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter++;
        printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
          "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
          aver_time_icp, aver_time_propag);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                     << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                     << " " << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        // dump_lio_state_to_log(fp);
      }
    }
    rate.sleep();
  }
  //--------------------------save map-----------------------------------
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performances
  if (!pcl_wait_save->empty() && pcd_save_en) {
    // string file_name = string("scans1.pcd");
    fs::path dir(save_path);

    int idx = 1;
    fs::path all_points_path;
  
    do {
      all_points_path = dir / ("scans" + std::to_string(idx) + ".pcd");
      ++idx;
    } while (fs::exists(all_points_path));
  
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_path.string(), *pcl_wait_save);
  }
  fout_out.close();
  fout_imu_pbp.close();
  return 0;
}