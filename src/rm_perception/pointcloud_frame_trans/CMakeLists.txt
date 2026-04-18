cmake_minimum_required(VERSION 3.5)
project(pointcloud_frame_trans)

# 查找ROS 2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pointcloud_frame_trans src/pointcloud_frame_trans.cpp)
include_directories(
  ${PCL_INCLUDE_DIRS}
)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 链接库
ament_target_dependencies(pointcloud_frame_trans
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  tf2_eigen
  PCL
  nav_msgs
)

# 安装可执行文件
install(TARGETS
  pointcloud_frame_trans
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件（如果有）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()