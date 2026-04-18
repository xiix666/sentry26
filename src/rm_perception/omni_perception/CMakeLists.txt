cmake_minimum_required(VERSION 3.8)
project(omni_perception)

# 查找ROS 2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

add_executable(omni_perception src/omni_perception.cpp)

# 链接库
ament_target_dependencies(omni_perception
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  rm_interfaces
)

# include_directories(${PROJECT_SOURCE_DIR}/include)

# 安装可执行文件
install(TARGETS
omni_perception
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