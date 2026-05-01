cmake_minimum_required(VERSION 3.8)
project(map_save)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(map_save src/map_save.cpp)
ament_target_dependencies(map_save rclcpp nav2_map_server nav2_msgs nav_msgs)

install(TARGETS
  map_save
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
