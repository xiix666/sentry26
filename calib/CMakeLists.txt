cmake_minimum_required(VERSION 3.8)
project(calib_multi_lidar)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Open3D REQUIRED)
find_package(message_filters REQUIRED)

add_executable(calib_multi_lidar src/calib.cpp)
ament_target_dependencies(calib_multi_lidar rclcpp PCL pcl_conversions Open3D message_filters)

install(TARGETS
        calib_multi_lidar
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
