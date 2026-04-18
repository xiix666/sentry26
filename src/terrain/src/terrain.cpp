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

#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

// 计时相关类型别名
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using DurationMs = std::chrono::duration<double, std::milli>; // 毫秒级时长

// 全局参数定义
double scanVoxelSize = 0.1;
double decayTime = 10.0;
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

// terrain voxel parameters
constexpr float terrainVoxelSize = 2.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
int terrainVoxelShiftX2 = 0;
int terrainVoxelShiftY2 = 0;
constexpr int terrainVoxelWidth = 41;

constexpr int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
constexpr int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;
constexpr int terrainVoxelWidth2 = terrainVoxelWidth / 2 + 1;
constexpr int terrainVoxelHalfWidth2 = (terrainVoxelWidth2 - 1) / 2;
constexpr float terrainVoxelSize2 = terrainVoxelSize / 2;
constexpr int kTerrainVoxelNum2 = terrainVoxelWidth2 * terrainVoxelWidth2;

double scanVoxelSize2;    // 局部点云下采样分辨率（=scanVoxelSize/2）
double voxelTimeUpdateThre2; // 局部体素更新时间阈值（=voxelTimeUpdateThre/2）
double terrainConnThre2;  // 局部地形连通性阈值（=terrainConnThre/2）

// planar voxel parameters
float planarVoxelSize = 0.4;
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
constexpr int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

// 话题名称
std::string cloud_in = "/livox/pointcloud2/transframe";
std::string odom_in = "lidar_odometry";
std::string odom_frame;
// 点云缓存
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>());

// 体素网格数组
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud2[kTerrainVoxelNum2];

// 体素更新计数和时间
int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
int terrainVoxelUpdateNum2[kTerrainVoxelNum2] = {0};
float terrainVoxelUpdateTime2[kTerrainVoxelNum2] = {0};

// 平面体素相关
float planarVoxelElev[kPlanarVoxelNum] = {0};
int planarVoxelConn[kPlanarVoxelNum] = {0};
std::vector<float> planarPointElev[kPlanarVoxelNum];
std::queue<int> planarVoxelQueue;

// 状态变量
double laserCloudTime = 0;
bool newlaserCloud = false;
double systemInitTime = 0;
bool systemInited = false;
double clear_interval = 0.8;
double last_clear = 0.0;

// 车辆位姿
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

// PCL滤波器和KD树
pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter2;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

// 计时辅助函数
inline double calcDurationMs(const TimePoint& start, const TimePoint& end) {
  return std::chrono::duration_cast<DurationMs>(end - start).count();
}

// 判断是否在局部体素区域内
bool isInLocalVoxelArea(float x, float y) { 
  float dis = sqrt((x - vehicleX) * (x - vehicleX) + (y - vehicleY) * (y - vehicleY));
  return dis <= localTerrainMapRadius;
}

// 里程计回调函数
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

// 激光点云回调函数
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();

  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  laserCloudCrop->reserve(laserCloud->size());
  int laserCloudSize = laserCloud->points.size();

  // 自动清理触发
  if (laserCloudTime - last_clear >= clear_interval) {
    clearingCloud = true;
    last_clear = laserCloudTime;
    RCLCPP_DEBUG(rclcpp::get_logger("terrainAnalysisExt"), 
                 "Auto trigger clearingCloud=true (interval: %.2fs)", clear_interval);
  }

  // 点云裁剪
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > lowerBoundZ - disRatioZ * dis &&
        pointZ - vehicleZ < upperBoundZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// // 局部地形点云回调函数
// void terrainCloudLocalHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudLocal2) {
//   terrainCloudLocal->clear();
//   pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
// }

// // 手柄回调函数
// void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
//   if (joy->buttons[5] > 0.5) {
//     clearingCloud = true;
//   }
// }

// 清理距离回调函数
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
  clearingDis = dis->data;
  clearingCloud = true;
}


double processTerrainAnalysis(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTerrainCloud,
                              rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTerrainCloudLocal) noexcept {
  TimePoint loop_start_time = Clock::now();
  
  // 1. 更新小地图体素偏移
  float terrainVoxelCenX2 = terrainVoxelSize2 * terrainVoxelShiftX2;
  float terrainVoxelCenY2 = terrainVoxelSize2 * terrainVoxelShiftY2;
  
  while (vehicleX - terrainVoxelCenX2 < -terrainVoxelSize2) {
    for (int indY = 0; indY < terrainVoxelWidth2; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud2[terrainVoxelWidth2 * (terrainVoxelWidth2 - 1) + indY];
      for (int indX = terrainVoxelWidth2 - 1; indX >= 1; indX--) {
        terrainVoxelCloud2[terrainVoxelWidth2 * indX + indY] =
            terrainVoxelCloud2[terrainVoxelWidth2 * (indX - 1) + indY];
      }
      terrainVoxelCloud2[indY] = terrainVoxelCloudPtr;
      terrainVoxelCloud2[indY]->clear();
    }
    terrainVoxelShiftX2--;
    terrainVoxelCenX2 = terrainVoxelSize2 * terrainVoxelShiftX2;
  }
  
  while (vehicleX - terrainVoxelCenX2 > terrainVoxelSize2) {
    for (int indY = 0; indY < terrainVoxelWidth2; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud2[indY];
      for (int indX = 0; indX < terrainVoxelWidth2 - 1; indX++) {
        terrainVoxelCloud2[terrainVoxelWidth2 * indX + indY] =
            terrainVoxelCloud2[terrainVoxelWidth2 * (indX + 1) + indY];
      }
      terrainVoxelCloud2[terrainVoxelWidth2 * (terrainVoxelWidth2 - 1) + indY] = terrainVoxelCloudPtr;
      terrainVoxelCloud2[terrainVoxelWidth2 * (terrainVoxelWidth2 - 1) + indY]->clear();
    }
    terrainVoxelShiftX2++;
    terrainVoxelCenX2 = terrainVoxelSize2 * terrainVoxelShiftX2;
  }
  
  while (vehicleY - terrainVoxelCenY2 < -terrainVoxelSize2) {
    for (int indX = 0; indX < terrainVoxelWidth2; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud2[terrainVoxelWidth2 * indX + (terrainVoxelWidth2 - 1)];
      for (int indY = terrainVoxelWidth2 - 1; indY >= 1; indY--) {
        terrainVoxelCloud2[terrainVoxelWidth2 * indX + indY] =
            terrainVoxelCloud2[terrainVoxelWidth2 * indX + (indY - 1)];
      }
      terrainVoxelCloud2[terrainVoxelWidth2 * indX] = terrainVoxelCloudPtr;
      terrainVoxelCloud2[terrainVoxelWidth2 * indX]->clear();
    }
    terrainVoxelShiftY2--;
    terrainVoxelCenY2 = terrainVoxelSize2 * terrainVoxelShiftY2;
  }
  
  while (vehicleY - terrainVoxelCenY2 > terrainVoxelSize2) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud2[terrainVoxelWidth2 * indX];
      for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
        terrainVoxelCloud2[terrainVoxelWidth2 * indX + indY] =
            terrainVoxelCloud2[terrainVoxelWidth2 * indX + (indY + 1)];
      }
      terrainVoxelCloud2[terrainVoxelWidth2 * indX + (terrainVoxelWidth2 - 1)] = terrainVoxelCloudPtr;
      terrainVoxelCloud2[terrainVoxelWidth2 * indX + (terrainVoxelWidth2 - 1)]->clear();
    }
    terrainVoxelShiftY2++;
    terrainVoxelCenY2 = terrainVoxelSize2 * terrainVoxelShiftY2;
  }

  // 2. 更新大地图体素偏移
  float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
  
  while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
    for (int indY = 0; indY < terrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
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
  
  while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
    for (int indY = 0; indY < terrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[indY];
      for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
      }
      terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
    }
    terrainVoxelShiftX++;
    terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  }
  
  while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
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
  
  while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * indX];
      for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
      }
      terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
    }
    terrainVoxelShiftY++;
    terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
  }

  // 3. 将裁剪后的点云存入体素网格
  pcl::PointXYZI point;
  int laserCloudCropSize = laserCloudCrop->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = laserCloudCrop->points[i];

    int indX = static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
    int indY = static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
    
    if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
      indX--;
    if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
      indY--;
    
    if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth) {
      terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
      terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
    }
  }

  // 4. 下采样并更新大地图体素点云
  for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
    if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
        laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre ||
        clearingCloud) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];

      laserCloudDwz->clear();
      downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
      downSizeFilter.filter(*laserCloudDwz);

      terrainVoxelCloudPtr->clear();
      int laserCloudDwzSize = laserCloudDwz->points.size();
      for (int i = 0; i < laserCloudDwzSize; i++) {
        point = laserCloudDwz->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
            (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) &&
            !(dis < clearingDis && clearingCloud)) {
          terrainVoxelCloudPtr->push_back(point);
        }
      }

      terrainVoxelUpdateNum[ind] = 0;
      terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
    }
  }

  // 5. 拼接地形点云
  terrainCloud->clear();
  int terrainCloudSize = 0;
  for (int indX = terrainVoxelHalfWidth - 10; indX <= terrainVoxelHalfWidth + 10; indX++) {
    for (int indY = terrainVoxelHalfWidth - 10; indY <= terrainVoxelHalfWidth + 10; indY++) {
      if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth) {
        terrainCloudSize += terrainVoxelCloud[terrainVoxelWidth * indX + indY]->size();
      }
    }
  }
  terrainCloudElev->reserve(terrainCloudSize);
  for (int indX = terrainVoxelHalfWidth - 10; indX <= terrainVoxelHalfWidth + 10; indX++) {
    for (int indY = terrainVoxelHalfWidth - 10; indY <= terrainVoxelHalfWidth + 10; indY++) {
      if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth) {
        *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
      }
    }
  }

  // 6. 计算平面体素高度
  for (int i = 0; i < kPlanarVoxelNum; i++) {
    planarVoxelElev[i] = 0;
    planarVoxelConn[i] = 0;
    planarPointElev[i].clear();
  }
  
  // int terrainCloudSize = terrainCloud->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrainCloud->points[i];
    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                     (point.y - vehicleY) * (point.y - vehicleY));
    if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
        point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {
      int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;
      if(point.z-vehicleZ > lowerBoundZ && point.z-vehicleZ < upperBoundZ){
        for (int dX = -1; dX <= 1; dX++) {
          for (int dY = -1; dY <= 1; dY++) {
            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                indY + dY >= 0 && indY + dY < planarVoxelWidth) {
              planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
            }
          }
        }
      }
    }
  }

  // 7. 计算每个平面体素的代表高度（排序取分位数或最小值）
  if (useSorting) {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planarPointElev[i].size();
      if (planarPointElevSize > 0) {
        sort(planarPointElev[i].begin(), planarPointElev[i].end());

        int quantileID = static_cast<int>(quantileZ * planarPointElevSize);
        quantileID = std::max(0, std::min(quantileID, planarPointElevSize - 1));

        planarVoxelElev[i] = planarPointElev[i][quantileID];
      }
    }
  } else {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planarPointElev[i].size();
      if (planarPointElevSize > 0) {
        float minZ = 1000.0;
        for (int j = 0; j < planarPointElevSize; j++) {
          minZ = std::min(minZ, planarPointElev[i][j]);
        }
        planarVoxelElev[i] = minZ;
      }
    }
  }

  // 8. 地形连通性检测（移除天花板）
  if (checkTerrainConn) {
    int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
    if (planarPointElev[ind].size() == 0)
      planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;

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
            if (planarVoxelConn[ind] == 0 && planarPointElev[ind].size() > 0) {
              if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) < terrainConnThre) {
                planarVoxelQueue.push(ind);
                planarVoxelConn[ind] = 1;
              } else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) > ceilingFilteringThre) {
                planarVoxelConn[ind] = -1;
              }
            }
          }
        }
      }
    }
  }

  // 9. 筛选最终的地形点云
  terrainCloudElev->clear();
  terrainCloudLocal->clear();
  int terrainCloudElevSize = 0;
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrainCloud->points[i];
    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                     (point.y - vehicleY) * (point.y - vehicleY));
    if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
        point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {
      int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;

      if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
        int ind = planarVoxelWidth * indX + indY;
        float disZ = fabs(point.z - planarVoxelElev[ind]);
        if (disZ < vehicleHeight && disZ > 0.1 &&
            (planarVoxelConn[ind] == 2 || !checkTerrainConn)) {
          terrainCloudElev->push_back(point);
          terrainCloudElev->points.back().intensity = disZ;
          terrainCloudElevSize++;

          if (dis <= localTerrainMapRadius) {
            terrainCloudLocal->push_back(point);
            terrainCloudLocal->points.back().intensity = disZ;
          }
        }
      }
    }
  }

  // 10. 发布地形点云
  clearingCloud = false;
  sensor_msgs::msg::PointCloud2 terrainCloud2;
  pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
  terrainCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
  terrainCloud2.header.frame_id = "odom";
  pubTerrainCloud->publish(terrainCloud2);

  sensor_msgs::msg::PointCloud2 terrainCloudLocalMsg;
  pcl::toROSMsg(*terrainCloudLocal, terrainCloudLocalMsg);
  terrainCloudLocalMsg.header.stamp = terrainCloud2.header.stamp; 
  terrainCloudLocalMsg.header.frame_id = terrainCloud2.header.frame_id; 
  pubTerrainCloudLocal->publish(terrainCloudLocalMsg);
  // 计算总耗时并返回
  double total_process_duration = calcDurationMs(loop_start_time, Clock::now());
  return total_process_duration;
}

int main(int argc, char **argv) {
  // 初始化ROS 2节点
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysisExt");

  // 声明并获取参数
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
  nh->declare_parameter<std::string>("cloud_in", cloud_in);
  nh->declare_parameter<std::string>("odom_in", odom_in);
  nh->declare_parameter<std::string>("odom_frame", odom_frame);
  // 获取参数值
  nh->get_parameter("odom_frame",odom_frame);
  nh->get_parameter("cloud_in", cloud_in);
  nh->get_parameter("odom_in", odom_in);
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

  // 计算派生参数
  scanVoxelSize2 = scanVoxelSize / 2.0;
  voxelTimeUpdateThre2 = voxelTimeUpdateThre / 2.0;
  terrainConnThre2 = terrainConnThre / 2.0;

  // 创建订阅器
  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in, 5, laserCloudHandler);

  // 创建发布器
  auto pubTerrainCloud =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_ext", 2);
  auto pubTerrainCloudLocal =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_local", 2);
  // 初始化体素点云指针
  for (int i = 0; i < kTerrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < kTerrainVoxelNum2; i++) {
    terrainVoxelCloud2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  
  // 初始化下采样滤波器
  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
  downSizeFilter2.setLeafSize(scanVoxelSize2, scanVoxelSize2, scanVoxelSize2);

  // 主循环
  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();

  while (status) {
    rclcpp::spin_some(nh);

    // 有新点云时调用地形分析函数
    if (newlaserCloud) {
      newlaserCloud = false;
      double process_time = processTerrainAnalysis(pubTerrainCloud, pubTerrainCloudLocal);
      
      // 输出耗时信息
      // std::cout << "=====================================" << std::endl;
      // std::cout << "[INFO] 单次地形分析总耗时: " << std::fixed << std::setprecision(2) 
      //           << process_time << " ms" << std::endl;
      // std::cout << "=====================================\n" << std::endl;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}