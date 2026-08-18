#include "li_initialization.h"
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false,
     refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0,
       online_calib_starts_time = 0.0;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::msg::Imu imu_last, imu_next;

PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

const double MAX_ANGULAR_VEL = 30.0;
const double MAX_LINEAR_ACC = 2.0 * 9.81;
const double QUAT_NORM_TOL = 0.1;
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{

  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;
      }
      if (frame_ct < 10) {
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;
        lidar_buffer.push_back(ptr_con_i);
        double time_con_i = time_con;
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;

}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{

  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  if (cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;
      }
      if (frame_ct < 10) {
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;
        double time_con_i = time_con;
        lidar_buffer.push_back(ptr_con_i);
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;

}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in)
{

  auto is_valid = [](double val) {
    return !std::isnan(val) && !std::isinf(val);
  };

 if (!is_valid(msg_in->angular_velocity.x) || !is_valid(msg_in->angular_velocity.y) || !is_valid(msg_in->angular_velocity.z) ||
 fabs(msg_in->angular_velocity.x) > MAX_ANGULAR_VEL ||
 fabs(msg_in->angular_velocity.y) > MAX_ANGULAR_VEL ||
 fabs(msg_in->angular_velocity.z) > MAX_ANGULAR_VEL) {
RCLCPP_WARN(rclcpp::get_logger("li_initialization"), "IMU angular velocity abnormal: x=%.2f, y=%.2f, z=%.2f",
           msg_in->angular_velocity.x, msg_in->angular_velocity.y, msg_in->angular_velocity.z);
return;
}

if (!is_valid(msg_in->linear_acceleration.x) || !is_valid(msg_in->linear_acceleration.y) || !is_valid(msg_in->linear_acceleration.z) ||
 fabs(msg_in->linear_acceleration.x) > MAX_LINEAR_ACC ||
 fabs(msg_in->linear_acceleration.y) > MAX_LINEAR_ACC ||
 fabs(msg_in->linear_acceleration.z) > MAX_LINEAR_ACC) {
RCLCPP_WARN(rclcpp::get_logger("li_initialization"), "IMU linear acceleration abnormal: x=%.2f, y=%.2f, z=%.2f",
           msg_in->linear_acceleration.x, msg_in->linear_acceleration.y, msg_in->linear_acceleration.z);
return;
}

  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  msg->header.stamp = get_ros_time(
    get_time_sec(msg_in->header.stamp) - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

  double timestamp = get_time_sec(msg->header.stamp);

  if (timestamp < last_timestamp_imu) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "imu loop back, clear deque");

    return;
  }
  imu_deque.emplace_back(msg);
  last_timestamp_imu = timestamp;

}

bool sync_packages(MeasureGroup & meas)
{
  {
    if (!imu_en) {
      if (!lidar_buffer.empty()) {
        if (!lidar_pushed) {
          meas.lidar = lidar_buffer.front();
          meas.lidar_beg_time = time_buffer.front();
          lose_lid = false;
          if (meas.lidar->points.empty()) {
            std::cout << "lose lidar" << '\n';

            lose_lid = true;
          } else {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt : meas.lidar->points) {
              if (pt.curvature > end_time) {
                end_time = pt.curvature;
              }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
          }
          lidar_pushed = true;
        }

        time_buffer.pop_front();
        lidar_buffer.pop_front();
        lidar_pushed = false;
        if (!lose_lid) {
          return true;
        } else {
          return false;
        }
      }
      return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty()) {
      return false;
    }
    if (!lidar_pushed) {
      lose_lid = false;
      meas.lidar = lidar_buffer.front();
      meas.lidar_beg_time = time_buffer.front();
      if (meas.lidar->points.size() < 1) {
        std::cout << "lose lidar" << '\n';
        lose_lid = true;

      } else {
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt : meas.lidar->points) {
          if (pt.curvature > end_time) {
            end_time = pt.curvature;
          }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        meas.lidar_last_time = lidar_end_time;
      }
      lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time)) {
      return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
      return false;
    }

    if (!lose_lid && !imu_pushed) {
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        imu_next = *(imu_deque.front());
        meas.imu.shrink_to_fit();
        while (imu_time < lidar_end_time) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    if (lose_lid && !imu_pushed) {
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();

        imu_next = *(imu_deque.front());
        while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
  }
}
