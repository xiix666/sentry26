cmake_minimum_required(VERSION 3.8)
project(angle_pub)

# 查找ROS 2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(angles REQUIRED)
add_executable(angle_pub src/angle_pub.cpp)

# 链接库
ament_target_dependencies(angle_pub
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  rm_interfaces
  angles
)

include_directories(${PROJECT_SOURCE_DIR}/include)

# 安装可执行文件
install(TARGETS
angle_pub
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