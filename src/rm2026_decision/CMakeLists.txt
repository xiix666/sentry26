cmake_minimum_required(VERSION 3.8)
project(rm2026_decision)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(behaviors conditions decorators ${behaviortree_cpp_INCLUDE_DIRS})

# 行为节点库
add_library(behavior_nodes SHARED src/behavior_nodes.cpp)
ament_target_dependencies(behavior_nodes
  rclcpp
  std_msgs
  rm_interfaces
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
  nav2_msgs
)

# 主决策节点
add_executable(rm2026_decision_node src/decision_node.cpp)
ament_target_dependencies(rm2026_decision_node
  rclcpp
  std_msgs
  rm_interfaces
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
)

target_link_libraries(rm2026_decision_node behavior_nodes)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  behavior_nodes
  rm2026_decision_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY behaviors/
  DESTINATION include/
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()