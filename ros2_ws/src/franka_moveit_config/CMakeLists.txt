cmake_minimum_required(VERSION 3.5)
project(franka_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/start_servo.py
    scripts/ps5_6dof_servo_teleop.py
    scripts/gamepad_gripper_teleop.py
    scripts/ps5_arm_home.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_srdf_tests test/srdf_tests.py)
endif()

ament_package()
