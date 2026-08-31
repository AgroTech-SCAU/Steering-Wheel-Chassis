cmake_minimum_required(VERSION 3.8)
project(vison_topic_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VisionDetect.srv"
  "msg/DetectionCenter.msg"
  "msg/DetectionCenterArray.msg"
  "msg/PickTarget.msg"
  DEPENDENCIES std_msgs
)

ament_package()
