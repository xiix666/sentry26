cmake_minimum_required(VERSION 3.8)
project(fake_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(fake_sub_node src/fake_sub_node.cpp)

# Link libraries
ament_target_dependencies(fake_sub_node
  rclcpp
  std_msgs
  rm_interfaces
  geometry_msgs
  tf2
  tf2_ros
)

# Install executable
install(TARGETS
  fake_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()