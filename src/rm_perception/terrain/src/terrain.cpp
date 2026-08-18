// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include <math.h>
#include <queue>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/features/normal_3d.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using DurationMs = std::chrono::duration<double, std::milli>;
double scanVoxelSize = 0.1;
double decayTime = 1.0;
double noDecayDis = 0;
double clearingDis = 30.0;
bool clearingCloud = true;
bool useSorting = false;
double quantileZ = 0.25;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double lowerBoundZ = -1.5;
double upperBoundZ = 1.0;
double disRatioZ = 0.1;
bool checkTerrainConn = true;
double terrainUnderVehicle = -0.75;
double terrainConnThre = 0.5;
double ceilingFilteringThre = 2.0;
double localTerrainMapRadius = 4.0;
constexpr float terrainVoxelSize = 2.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
constexpr int terrainVoxelWidth = 41;
constexpr int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
constexpr int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;
float planarVoxelSize = 0.4;
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
constexpr int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;
std::string cloud_in = "/livox/pointcloud2/transframe";
std::string odom_in = "lidar_odometry";
std::string odom_frame;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloud[kTerrainVoxelNum];
int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
float planarVoxelElev[kPlanarVoxelNum] = {0};
int planarVoxelConn[kPlanarVoxelNum] = {0};
bool planarVoxelHasPoint[kPlanarVoxelNum] = {false};
std::vector<float> planarPointElev[kPlanarVoxelNum];
std::queue<int> planarVoxelQueue;
double laserCloudTime = 0;
bool newlaserCloud = false;
double systemInitTime = 0;
bool systemInited = false;
double clear_interval = 1.0;
double last_clear = 0.0;
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float cloudVehicleX = 0.0F;
float cloudVehicleY = 0.0F;
float cloudVehicleZ = 0.0F;
float cloudVehicleRoll = 0.0F;
float cloudVehiclePitch = 0.0F;
float cloudVehicleYaw = 0.0F;
pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilter;
pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    laserCloudCropDownsampled(new pcl::PointCloud<pcl::PointXYZINormal>());
bool useVoxelPCA = false;
bool useInputDownsampling = false;
double inputDownsampleLeafSize = 0.10;
constexpr double normalVoxelSize = 0.20;
constexpr int normalVoxelNeighborRadius = 1;
constexpr int normalVoxelMinPoints = 10;

struct NormalVoxelKey {
  int x;
  int y;
  int z;
  bool operator==(const NormalVoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct NormalVoxelKeyHash {
  std::size_t operator()(const NormalVoxelKey &key) const {
    std::size_t seed = 0;
    auto combine = [&seed](int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    };

    combine(key.x);
    combine(key.y);
    combine(key.z);

    return seed;
  }
};

struct NormalVoxelStats {
  int count{0};
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();
  bool normal_computed{false};
  bool normal_valid{false};
  Eigen::Vector3d normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  float gradient{0.0F};
};
inline double calcDurationMs(const TimePoint &start, const TimePoint &end) {
  return std::chrono::duration_cast<DurationMs>(end - start).count();
}

inline NormalVoxelKey getNormalVoxelKey(double x, double y, double z) {
  const int voxel_x = static_cast<int>(std::floor(x / normalVoxelSize));
  const int voxel_y = static_cast<int>(std::floor(y / normalVoxelSize));
  const int voxel_z = static_cast<int>(std::floor(z / normalVoxelSize));
  return NormalVoxelKey{voxel_x, voxel_y, voxel_z};
}

bool isInLocalVoxelArea(float x, float y) {
  float dis = sqrt((x - cloudVehicleX) * (x - cloudVehicleX) +
                   (y - cloudVehicleY) * (y - cloudVehicleY));
  return dis <= localTerrainMapRadius;
}

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
  cloudVehicleX = vehicleX;
  cloudVehicleY = vehicleY;
  cloudVehicleZ = vehicleZ;

  cloudVehicleRoll = vehicleRoll;
  cloudVehiclePitch = vehiclePitch;
  cloudVehicleYaw = vehicleYaw;
  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);
  pcl::PointXYZINormal point;
  laserCloudCrop->clear();
  laserCloudCrop->reserve(laserCloud->size());
  int laserCloudSize = laserCloud->points.size();

  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];
    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;
    float dis = sqrt((pointX - cloudVehicleX) * (pointX - cloudVehicleX) +
                     (pointY - cloudVehicleY) * (pointY - cloudVehicleY));
    if (pointZ - cloudVehicleZ > lowerBoundZ - disRatioZ * dis &&
        pointZ - cloudVehicleZ < upperBoundZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  if (useInputDownsampling && !laserCloudCrop->empty()) {
    laserCloudCropDownsampled->clear();

    downSizeFilter.setInputCloud(laserCloudCrop);
    downSizeFilter.filter(*laserCloudCropDownsampled);

    laserCloudCrop->swap(*laserCloudCropDownsampled);
  }

  newlaserCloud = true;
}

void terrainCloudLocalHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudLocal2) {
  terrainCloudLocal->clear();
  pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
  if (joy->buttons[5] > 0.5) {
    clearingCloud = true;
  }
}

void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
  clearingDis = dis->data;
  clearingCloud = true;
}

double processTerrainAnalysis(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTerrainCloud,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubTerrainCloudLocal) noexcept {
  TimePoint loop_start_time = Clock::now();
  float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

  while (cloudVehicleX - terrainVoxelCenX < -terrainVoxelSize) {
    for (int indY = 0; indY < terrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
      for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
      }
      terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
      terrainVoxelCloud[indY]->clear();
    }
    terrainVoxelShiftX--;
    terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  }

  while (cloudVehicleX - terrainVoxelCenX > terrainVoxelSize) {
    for (int indY = 0; indY < terrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[indY];
      for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
      }
      terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] =
          terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
          ->clear();
    }
    terrainVoxelShiftX++;
    terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  }

  while (cloudVehicleY - terrainVoxelCenY < -terrainVoxelSize) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
      for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
      }
      terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
    }
    terrainVoxelShiftY--;
    terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
  }

  while (cloudVehicleY - terrainVoxelCenY > terrainVoxelSize) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * indX];
      for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
      }
      terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] =
          terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
          ->clear();
    }
    terrainVoxelShiftY++;
    terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
  }

  pcl::PointXYZINormal point;
  int laserCloudCropSize = laserCloudCrop->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = laserCloudCrop->points[i];
    int indX =
        static_cast<int>((point.x - cloudVehicleX + terrainVoxelSize / 2) /
                         terrainVoxelSize) +
        terrainVoxelHalfWidth;
    int indY =
        static_cast<int>((point.y - cloudVehicleY + terrainVoxelSize / 2) /
                         terrainVoxelSize) +
        terrainVoxelHalfWidth;

    if (point.x - cloudVehicleX + terrainVoxelSize / 2 < 0)
      indX--;
    if (point.y - cloudVehicleY + terrainVoxelSize / 2 < 0)
      indY--;

    if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
        indY < terrainVoxelWidth) {
      terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
      terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
    }
  }

  const float current_relative_time =
      static_cast<float>(laserCloudTime - systemInitTime);

  for (int ind = 0; ind < kTerrainVoxelNum; ++ind) {
    auto &voxel_cloud = *terrainVoxelCloud[ind];
    auto &points = voxel_cloud.points;

    points.erase(
        std::remove_if(
            points.begin(), points.end(),
            [&](const pcl::PointXYZINormal &point) {
              if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                  !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
                return true;
              }

              const float point_age = current_relative_time - point.intensity;

              return point_age > static_cast<float>(decayTime);
            }),
        points.end());

    voxel_cloud.width = static_cast<std::uint32_t>(points.size());

    voxel_cloud.height = 1;
    voxel_cloud.is_dense = true;

    terrainVoxelUpdateNum[ind] = 0;
  }

  terrainCloud->clear();
  int terrainCloudSize = 0;
  for (int indX = terrainVoxelHalfWidth - 10;
       indX <= terrainVoxelHalfWidth + 10; indX++) {
    for (int indY = terrainVoxelHalfWidth - 10;
         indY <= terrainVoxelHalfWidth + 10; indY++) {
      if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
          indY < terrainVoxelWidth) {
        terrainCloudSize +=
            terrainVoxelCloud[terrainVoxelWidth * indX + indY]->size();
      }
    }
  }

  terrainCloudElev->reserve(terrainCloudSize);
  for (int indX = terrainVoxelHalfWidth - 10;
       indX <= terrainVoxelHalfWidth + 10; indX++) {
    for (int indY = terrainVoxelHalfWidth - 10;
         indY <= terrainVoxelHalfWidth + 10; indY++) {
      if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
          indY < terrainVoxelWidth) {
        *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
      }
    }
  }

  for (int i = 0; i < kPlanarVoxelNum; ++i) {
    planarVoxelElev[i] = 0.0F;
    planarVoxelConn[i] = 0;
    planarVoxelHasPoint[i] = false;
    if (useSorting) {
      planarPointElev[i].clear();
    }
  }

  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrainCloud->points[i];
    float dis = sqrt((point.x - cloudVehicleX) * (point.x - cloudVehicleX) +
                     (point.y - cloudVehicleY) * (point.y - cloudVehicleY));
    if (point.z - cloudVehicleZ > lowerBoundZ - disRatioZ * dis &&
        point.z - cloudVehicleZ < upperBoundZ + disRatioZ * dis) {
      int indX =
          static_cast<int>((point.x - cloudVehicleX + planarVoxelSize / 2) /
                           planarVoxelSize) +
          planarVoxelHalfWidth;
      int indY =
          static_cast<int>((point.y - cloudVehicleY + planarVoxelSize / 2) /
                           planarVoxelSize) +
          planarVoxelHalfWidth;

      if (point.x - cloudVehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - cloudVehicleY + planarVoxelSize / 2 < 0)
        indY--;
      if (point.z - cloudVehicleZ > lowerBoundZ &&
          point.z - cloudVehicleZ < upperBoundZ && std::isfinite(point.z)) {
        for (int dX = -1; dX <= 1; ++dX) {
          for (int dY = -1; dY <= 1; ++dY) {
            const int voxel_x = indX + dX;
            const int voxel_y = indY + dY;

            if (voxel_x < 0 || voxel_x >= planarVoxelWidth || voxel_y < 0 ||
                voxel_y >= planarVoxelWidth) {
              continue;
            }

            const int voxel_index = planarVoxelWidth * voxel_x + voxel_y;

            if (useSorting) {
              planarPointElev[voxel_index].push_back(point.z);
            } else {
              if (!planarVoxelHasPoint[voxel_index]) {
                planarVoxelElev[voxel_index] = point.z;
              } else {
                planarVoxelElev[voxel_index] =
                    std::min(planarVoxelElev[voxel_index], point.z);
              }
            }

            planarVoxelHasPoint[voxel_index] = true;
          }
        }
      }
    }
  }

  if (useSorting) {
    const double valid_quantile = std::clamp(quantileZ, 0.0, 0.20);
    constexpr int kMinQuantilePoints = 5;
    constexpr float kMaxRiseFromMinimum = 0.15F;

    for (int i = 0; i < kPlanarVoxelNum; ++i) {
      auto &elevations = planarPointElev[i];

      elevations.erase(
          std::remove_if(elevations.begin(), elevations.end(),
                         [](float z) { return !std::isfinite(z); }),
          elevations.end());
      const int point_count = static_cast<int>(elevations.size());

      if (point_count <= 0) {
        continue;
      }

      const auto min_iterator =
          std::min_element(elevations.begin(), elevations.end());
      const float minimum_z = *min_iterator;

      if (point_count < kMinQuantilePoints) {
        planarVoxelElev[i] = minimum_z;

        continue;
      }

      const std::size_t quantile_index = static_cast<std::size_t>(
          std::floor(valid_quantile * static_cast<double>(point_count - 1)));
      auto quantile_iterator = elevations.begin() + quantile_index;
      std::nth_element(elevations.begin(), quantile_iterator, elevations.end());
      const float quantile_height = elevations[quantile_index];

      planarVoxelElev[i] =
          std::min(quantile_height, minimum_z + kMaxRiseFromMinimum);
    }
  }

  if (checkTerrainConn) {
    int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
    const float expected_ground_z =
        cloudVehicleZ + static_cast<float>(terrainUnderVehicle);
    constexpr float kMaxCenterGroundError = 0.40F;
    const bool center_elevation_invalid =
        !planarVoxelHasPoint[ind] || !std::isfinite(planarVoxelElev[ind]) ||
        std::abs(planarVoxelElev[ind] - expected_ground_z) >
            kMaxCenterGroundError;

    if (center_elevation_invalid) {
      planarVoxelElev[ind] = expected_ground_z;
    }

    planarVoxelQueue.push(ind);
    planarVoxelConn[ind] = 1;

    while (!planarVoxelQueue.empty()) {
      int front = planarVoxelQueue.front();
      planarVoxelConn[front] = 2;
      planarVoxelQueue.pop();
      int indX = static_cast<int>(front / planarVoxelWidth);
      int indY = front % planarVoxelWidth;

      for (int dX = -10; dX <= 10; dX++) {
        for (int dY = -10; dY <= 10; dY++) {
          if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
              indY + dY >= 0 && indY + dY < planarVoxelWidth) {
            ind = planarVoxelWidth * (indX + dX) + indY + dY;
            if (planarVoxelConn[ind] == 0 && planarVoxelHasPoint[ind]) {
              if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) <
                  terrainConnThre) {
                planarVoxelQueue.push(ind);
                planarVoxelConn[ind] = 1;
              } else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) >
                         ceilingFilteringThre) {
                planarVoxelConn[ind] = -1;
              }
            }
          }
        }
      }
    }
  }

  terrainCloudElev->clear();
  terrainCloudLocal->clear();
  int terrainCloudElevSize = 0;
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrainCloud->points[i];
    float dis = sqrt((point.x - cloudVehicleX) * (point.x - cloudVehicleX) +
                     (point.y - cloudVehicleY) * (point.y - cloudVehicleY));
    if (point.z - cloudVehicleZ > lowerBoundZ - disRatioZ * dis &&
        point.z - cloudVehicleZ < upperBoundZ + disRatioZ * dis) {
      int indX =
          static_cast<int>((point.x - cloudVehicleX + planarVoxelSize / 2) /
                           planarVoxelSize) +
          planarVoxelHalfWidth;
      int indY =
          static_cast<int>((point.y - cloudVehicleY + planarVoxelSize / 2) /
                           planarVoxelSize) +
          planarVoxelHalfWidth;

      if (point.x - cloudVehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - cloudVehicleY + planarVoxelSize / 2 < 0)
        indY--;

      if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
          indY < planarVoxelWidth) {
        int ind = planarVoxelWidth * indX + indY;
        float disZ = fabs(point.z - planarVoxelElev[ind]);
        if (disZ < vehicleHeight && disZ > 0.1 &&
            (planarVoxelConn[ind] == 2 || !checkTerrainConn)) {
          terrainCloudElev->push_back(point);
          auto &output_point = terrainCloudElev->points.back();
          output_point.intensity = disZ;

          output_point.normal_x = 0.0F;
          output_point.normal_y = 0.0F;
          output_point.normal_z = 1.0F;
          output_point.curvature = 0.0F;

          terrainCloudElevSize++;
        }
      }
    }
  }

  if (useVoxelPCA && !terrainCloudElev->empty() && !terrainCloud->empty()) {
    using NormalVoxelMap = std::unordered_map<NormalVoxelKey, NormalVoxelStats,
                                              NormalVoxelKeyHash>;

    NormalVoxelMap normal_voxels;
    normal_voxels.reserve(std::max<std::size_t>(128, terrainCloud->size() / 4));

    for (const auto &p : terrainCloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      const NormalVoxelKey key = getNormalVoxelKey(p.x, p.y, p.z);
      auto &voxel = normal_voxels[key];
      const Eigen::Vector3d v(p.x, p.y, p.z);

      ++voxel.count;
      voxel.sum += v;
      voxel.sum_outer += v * v.transpose();
    }

    for (auto &p : terrainCloudElev->points) {
      const NormalVoxelKey center_key = getNormalVoxelKey(p.x, p.y, p.z);
      auto center_it = normal_voxels.find(center_key);

      if (center_it == normal_voxels.end()) {
        p.normal_x = 0.0F;
        p.normal_y = 0.0F;
        p.normal_z = 1.0F;
        p.curvature = 0.0F;
        continue;
      }

      auto &center_voxel = center_it->second;

      if (!center_voxel.normal_computed) {
        center_voxel.normal_computed = true;
        int total_count = 0;
        Eigen::Vector3d total_sum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d total_sum_outer = Eigen::Matrix3d::Zero();

        for (int dx = -normalVoxelNeighborRadius;
             dx <= normalVoxelNeighborRadius; ++dx) {
          for (int dy = -normalVoxelNeighborRadius;
               dy <= normalVoxelNeighborRadius; ++dy) {
            for (int dz = -normalVoxelNeighborRadius;
                 dz <= normalVoxelNeighborRadius; ++dz) {
              const NormalVoxelKey neighbor_key{
                  center_key.x + dx, center_key.y + dy, center_key.z + dz};
              const auto neighbor_it = normal_voxels.find(neighbor_key);
              if (neighbor_it == normal_voxels.end()) {
                continue;
              }

              const auto &neighbor = neighbor_it->second;
              total_count += neighbor.count;
              total_sum += neighbor.sum;
              total_sum_outer += neighbor.sum_outer;
            }
          }
        }

        if (total_count >= normalVoxelMinPoints) {
          const double inv_count = 1.0 / static_cast<double>(total_count);
          const Eigen::Vector3d mean = total_sum * inv_count;
          Eigen::Matrix3d covariance =
              total_sum_outer * inv_count - mean * mean.transpose();

          covariance = 0.5 * (covariance + covariance.transpose());
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

          if (solver.info() == Eigen::Success) {
            Eigen::Vector3d normal = solver.eigenvectors().col(0);

            if (normal.allFinite() && normal.norm() > 1e-9) {
              normal.normalize();

              if (normal.z() < 0.0) {
                normal = -normal;
              }

              const double nx = normal.x();
              const double ny = normal.y();
              const double nz = normal.z();
              const double nz_sq = nz * nz;
              double gradient = (nx * nx + ny * ny) / std::max(nz_sq, 1e-4);

              gradient = std::min(gradient, 100.0);

              center_voxel.normal = normal;
              center_voxel.gradient = static_cast<float>(gradient);
              center_voxel.normal_valid = true;
            }
          }
        }
      }

      if (center_voxel.normal_valid) {
        p.normal_x = static_cast<float>(center_voxel.normal.x());
        p.normal_y = static_cast<float>(center_voxel.normal.y());
        p.normal_z = static_cast<float>(center_voxel.normal.z());
        p.curvature = center_voxel.gradient;
      } else {
        p.normal_x = 0.0F;
        p.normal_y = 0.0F;
        p.normal_z = 1.0F;
        p.curvature = 0.0F;
      }
    }

  } else if (!useVoxelPCA && !terrainCloudElev->empty() &&
             !terrainCloud->empty()) {
    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_full(
        new pcl::search::KdTree<pcl::PointXYZINormal>());

    ne.setInputCloud(terrainCloud);
    ne.setSearchMethod(tree_full);
    ne.setKSearch(15);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals_full(
        new pcl::PointCloud<pcl::PointXYZINormal>());
    ne.compute(*cloud_normals_full);
    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree_full;
    kdtree_full.setInputCloud(terrainCloud);
    const int knn = 1;
    std::vector<int> idx(knn);
    std::vector<float> dist(knn);

    for (size_t i = 0; i < terrainCloudElev->size(); ++i) {
      auto &p = terrainCloudElev->points[i];

      if (kdtree_full.nearestKSearch(p, knn, idx, dist) > 0) {
        const int original_idx = idx[0];
        const auto &normal = cloud_normals_full->points[original_idx];

        p.normal_x = normal.normal_x;
        p.normal_y = normal.normal_y;
        p.normal_z = normal.normal_z;

        if (fabs(normal.normal_z) < 1e-6f) {
          p.curvature = 0.0f;
          continue;
        }

        const float nx = normal.normal_x;
        const float ny = normal.normal_y;
        const float nz = normal.normal_z;
        const float gradient = (nx * nx + ny * ny) / (nz * nz);

        p.curvature = gradient;
      } else {
        p.normal_x = 0.0F;
        p.normal_y = 0.0F;
        p.normal_z = 1.0F;
        p.curvature = 0.0F;
      }
    }
  }

  for (std::size_t i = 0; i < terrainCloudElev->size(); i++) {
    auto &p = terrainCloudElev->points[i];
    float dis = sqrt((p.x - cloudVehicleX) * (p.x - cloudVehicleX) +
                     (p.y - cloudVehicleY) * (p.y - cloudVehicleY));
    if (dis <= localTerrainMapRadius) {
      terrainCloudLocal->push_back(p);
    }
  }
  clearingCloud = false;
  sensor_msgs::msg::PointCloud2 terrainCloud2;
  pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
  terrainCloud2.header.stamp =
      rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
  terrainCloud2.header.frame_id = "odom";
  pubTerrainCloud->publish(terrainCloud2);
  sensor_msgs::msg::PointCloud2 terrainCloudLocalMsg;
  pcl::toROSMsg(*terrainCloudLocal, terrainCloudLocalMsg);
  terrainCloudLocalMsg.header.stamp = terrainCloud2.header.stamp;
  terrainCloudLocalMsg.header.frame_id = terrainCloud2.header.frame_id;
  pubTerrainCloudLocal->publish(terrainCloudLocalMsg);
  double total_process_duration = calcDurationMs(loop_start_time, Clock::now());
  return total_process_duration;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysisExt");

  nh->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
  nh->declare_parameter<double>("decayTime", decayTime);
  nh->declare_parameter<double>("noDecayDis", noDecayDis);
  nh->declare_parameter<double>("clearingDis", clearingDis);
  nh->declare_parameter<bool>("useSorting", useSorting);
  nh->declare_parameter<double>("quantileZ", quantileZ);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->declare_parameter<double>("lowerBoundZ", lowerBoundZ);
  nh->declare_parameter<double>("upperBoundZ", upperBoundZ);
  nh->declare_parameter<double>("disRatioZ", disRatioZ);
  nh->declare_parameter<bool>("checkTerrainConn", checkTerrainConn);
  nh->declare_parameter<double>("terrainUnderVehicle", terrainUnderVehicle);
  nh->declare_parameter<double>("terrainConnThre", terrainConnThre);
  nh->declare_parameter<double>("ceilingFilteringThre", ceilingFilteringThre);
  nh->declare_parameter<double>("localTerrainMapRadius", localTerrainMapRadius);
  nh->declare_parameter<double>("clear_interval", clear_interval);
  nh->declare_parameter<std::string>("cloud_in", cloud_in);
  nh->declare_parameter<std::string>("odom_in", odom_in);
  nh->declare_parameter<std::string>("odom_frame", odom_frame);
  nh->declare_parameter<bool>("useVoxelPCA", useVoxelPCA);
  nh->declare_parameter<bool>("useInputDownsampling", useInputDownsampling);
  nh->declare_parameter<double>("inputDownsampleLeafSize",
                                inputDownsampleLeafSize);
  nh->get_parameter("odom_frame", odom_frame);
  nh->get_parameter("cloud_in", cloud_in);
  nh->get_parameter("odom_in", odom_in);
  nh->get_parameter("clear_interval", clear_interval);
  nh->get_parameter("scanVoxelSize", scanVoxelSize);
  nh->get_parameter("decayTime", decayTime);
  nh->get_parameter("noDecayDis", noDecayDis);
  nh->get_parameter("clearingDis", clearingDis);
  nh->get_parameter("useSorting", useSorting);
  nh->get_parameter("quantileZ", quantileZ);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->get_parameter("lowerBoundZ", lowerBoundZ);
  nh->get_parameter("upperBoundZ", upperBoundZ);
  nh->get_parameter("disRatioZ", disRatioZ);
  nh->get_parameter("checkTerrainConn", checkTerrainConn);
  nh->get_parameter("terrainUnderVehicle", terrainUnderVehicle);
  nh->get_parameter("terrainConnThre", terrainConnThre);
  nh->get_parameter("ceilingFilteringThre", ceilingFilteringThre);
  nh->get_parameter("localTerrainMapRadius", localTerrainMapRadius);
  nh->get_parameter("useVoxelPCA", useVoxelPCA);
  nh->get_parameter("useInputDownsampling", useInputDownsampling);
  nh->get_parameter("inputDownsampleLeafSize", inputDownsampleLeafSize);

  inputDownsampleLeafSize = std::max(0.01, inputDownsampleLeafSize);
  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
      odom_in, 5, odometryHandler);
  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in, 5, laserCloudHandler);
  auto pubTerrainCloud =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_ext", 2);
  auto pubTerrainCloudLocal =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>(
          "terrain_map_local", 2);
  for (int i = 0; i < kTerrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
  }

  const float input_leaf_size = static_cast<float>(inputDownsampleLeafSize);
  downSizeFilter.setLeafSize(input_leaf_size, input_leaf_size, input_leaf_size);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();

  while (status) {
    rclcpp::spin_some(nh);

    if (newlaserCloud) {
      newlaserCloud = false;
      processTerrainAnalysis(pubTerrainCloud, pubTerrainCloudLocal);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
