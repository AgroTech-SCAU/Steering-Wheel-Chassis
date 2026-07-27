cmake_minimum_required(VERSION 3.8)
project(send_navigation_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成自定义服务 NavigateToTarget.srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToTarget.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(send_nav_target_exe src/send_navigation_target.cpp)
# CMake target 名不能与 rosidl_generate_interfaces 的 ${PROJECT_NAME} 冲突，
# 通过 OUTPUT_NAME 将最终二进制文件指定为 send_navigation_target
set_target_properties(send_nav_target_exe PROPERTIES OUTPUT_NAME send_navigation_target)
target_include_directories(send_nav_target_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(send_nav_target_exe PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  send_nav_target_exe
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "geometry_msgs"
)

# 链接生成的服务接口到可执行文件
rosidl_target_interfaces(send_nav_target_exe
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS send_nav_target_exe
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
