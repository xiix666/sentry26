cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/AngleMsg.msg"
  "msg/AnglePubMsg.msg"
  "msg/CameraMsg.msg"
  "msg/EnemyDetectionMsg.msg"
  "msg/EnergyMsg.msg"
  "msg/LidarMsg.msg"
  "msg/NavMsg.msg"
  "msg/ReceiveMsg.msg"
  "msg/SendToLidarMsg.msg"
  "msg/ShootMsg.msg"
  "msg/ShootOnly.msg"
  "msg/ShootPriority.msg"
  "msg/SpecialMsg.msg"
  "msg/ReceiveLLC.msg"
  "msg/StatusMsg.msg"
  DEPENDENCIES geometry_msgs
)

ament_auto_package()
