cmake_minimum_required(VERSION 3.8)
project(atlas_mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/ManipulationStatus.msg"
  "srv/DetectCameraTarget.srv"
  "srv/StartNavigation.srv"
  "srv/CancelNavigation.srv"
  "srv/StartManipulation.srv"
  "srv/CancelManipulation.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
