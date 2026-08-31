cmake_minimum_required(VERSION 3.8)
project(atlas_competition_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/competition_preflight.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
