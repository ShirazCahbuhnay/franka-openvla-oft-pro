cmake_minimum_required(VERSION 3.8)
project(panda_gazebo_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/apply_table_collision_object.py
    scripts/joint_state_combiner.py
    scripts/start_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
