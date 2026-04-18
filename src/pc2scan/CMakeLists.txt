cmake_minimum_required(VERSION 3.8)
project(pc2scan)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pc2scan SHARED
  src/pc2scan.cpp)

rclcpp_components_register_node(pc2scan #库名称
PLUGIN "pc2scan::pc2scan"      #命名空间：类名
EXECUTABLE pc2scan_node)           #创建独立的可执行文件

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
