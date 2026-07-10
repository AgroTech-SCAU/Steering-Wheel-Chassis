cmake_minimum_required(VERSION 3.8)
project(atlas_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mcu_comm_bridge REQUIRED)
find_package(atlas_mission_interfaces REQUIRED)

add_executable(atlas_mission_manager_node
  src/main.cpp
  src/mission_state.cpp
  src/mcu_state_cache.cpp
  src/safety_controller.cpp
  src/mission_result_reporter.cpp
  src/mission_manager_node.cpp
)

target_include_directories(atlas_mission_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(atlas_mission_manager_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  mcu_comm_bridge
  atlas_mission_interfaces
)

target_link_libraries(atlas_mission_manager_node
  yaml-cpp
)

install(TARGETS atlas_mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
