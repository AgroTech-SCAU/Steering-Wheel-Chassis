cmake_minimum_required(VERSION 3.8)
project(vison_topic_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VisionDetect.srv"
)

ament_package()
