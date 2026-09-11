cmake_minimum_required(VERSION 3.8)
project(atlas_competition_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/arm_motion_calibration.py
  scripts/navigation_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/arm_motion_calibration_model.py
  scripts/navigation_calibration_model.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(arm_motion_calibration_model_test
    test/test_arm_motion_calibration_model.py
  )
  ament_add_pytest_test(navigation_calibration_model_test
    test/test_navigation_calibration_model.py
  )
  ament_add_pytest_test(navigation_calibration_runtime_source_test
    test/test_navigation_calibration_runtime_source.py
  )
endif()

ament_package()
