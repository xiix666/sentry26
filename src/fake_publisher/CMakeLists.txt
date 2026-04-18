cmake_minimum_required(VERSION 3.8)
project(fake_publisher)

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
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(fake_publisher_node src/fake_publisher_node.cpp)

# Link libraries
ament_target_dependencies(fake_publisher_node
  rclcpp
  std_msgs
  rm_interfaces
  geometry_msgs
  tf2
  tf2_ros
)

# Link yaml-cpp
target_link_libraries(fake_publisher_node yaml-cpp)

# Install executable
install(TARGETS
  fake_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()