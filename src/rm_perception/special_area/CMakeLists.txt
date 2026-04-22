cmake_minimum_required(VERSION 3.8)
project(special_area)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/special_area.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  std_msgs
  tf2_geometry_msgs
  rclcpp_components
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "SpecialArea"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()