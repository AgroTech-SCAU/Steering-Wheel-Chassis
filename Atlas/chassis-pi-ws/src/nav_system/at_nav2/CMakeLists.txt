cmake_minimum_required(VERSION 3.16)
project(at_nav2)

find_package(ament_cmake REQUIRED)

set(_install_dirs "")

foreach(dir launch config maps rviz2 )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    list(APPEND _install_dirs "${dir}")
  endif()
endforeach()

install(DIRECTORY rviz2/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(_install_dirs)
  install(DIRECTORY
    ${_install_dirs}
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
