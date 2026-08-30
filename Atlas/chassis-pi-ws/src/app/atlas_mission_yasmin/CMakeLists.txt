cmake_minimum_required(VERSION 3.10)
project(atlas_mission_yasmin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mcu_comm_bridge REQUIRED)
find_package(atlas_mission_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)

add_library(atlas_mission_yasmin_runtime
  src/competition_model.cpp
  src/machine.cpp
  src/runtime.cpp
  src/states.cpp
)

target_include_directories(atlas_mission_yasmin_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(atlas_mission_yasmin_runtime
  rclcpp
  geometry_msgs
  std_srvs
  mcu_comm_bridge
  atlas_mission_interfaces
)

target_link_libraries(atlas_mission_yasmin_runtime
  yaml-cpp
  yasmin::yasmin
)

add_executable(atlas_mission_yasmin_node
  src/atlas_mission_yasmin_node.cpp
)

ament_target_dependencies(atlas_mission_yasmin_node
  rclcpp
  geometry_msgs
  std_srvs
  mcu_comm_bridge
  atlas_mission_interfaces
)

target_link_libraries(atlas_mission_yasmin_node
  atlas_mission_yasmin_runtime
  yaml-cpp
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  yasmin_viewer::yasmin_viewer
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS atlas_mission_yasmin_runtime
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS atlas_mission_yasmin_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/mock_mcu.py
  scripts/mock_backends.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(competition_model_unit_test
    test/competition_model_unit_test.cpp
  )
  target_link_libraries(competition_model_unit_test
    atlas_mission_yasmin_runtime
  )

  ament_add_gtest(machine_unit_test
    test/machine_unit_test.cpp
  )
  target_link_libraries(machine_unit_test
    atlas_mission_yasmin_runtime
    yasmin::yasmin
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_srvs
  mcu_comm_bridge
  atlas_mission_interfaces
)

ament_package()
