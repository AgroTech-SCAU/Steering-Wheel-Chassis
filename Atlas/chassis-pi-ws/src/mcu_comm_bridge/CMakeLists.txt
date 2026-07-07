cmake_minimum_required(VERSION 3.8)
project(mcu_comm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoTaskEvent.msg"
  "msg/McuStatus.msg"
  "srv/Estop.srv"
  "srv/ReportMissionResult.srv"
  "srv/SetArmJoints.srv"
  "srv/SetArmOrientation.srv"
  "srv/SetArmPose.srv"
  "srv/SetArmPosition.srv"
  "srv/SetYawTarget.srv"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_library(mcu_comm_bridge_codec
  src/auto_task_latch_tracker.cpp
  src/binary_frame.cpp
  src/arm_state_codec.cpp
  src/mcu_status_codec.cpp
)

target_include_directories(mcu_comm_bridge_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(mcu_comm_bridge_node
  src/mcu_comm_bridge_node.cpp
  src/publish_scheduler.cpp
  src/serial_port.cpp
)

ament_target_dependencies(mcu_comm_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

target_link_libraries(mcu_comm_bridge_node
  mcu_comm_bridge_codec
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(mcu_comm_bridge_node "${cpp_typesupport_target}")

target_include_directories(mcu_comm_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS mcu_comm_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mcu_comm_bridge_codec
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
