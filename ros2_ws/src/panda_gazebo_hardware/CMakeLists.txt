cmake_minimum_required(VERSION 3.8)
project(panda_gazebo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)

set(GZ_SIM_TARGET "")

if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_TARGET gz-sim8::core)
  add_definitions(-DGZ_HEADERS)
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_TARGET ignition-gazebo6::core)
endif()

add_library(${PROJECT_NAME} SHARED
  src/panda_gravity_compensated_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${GZ_SIM_TARGET}
  orocos-kdl
)

ament_target_dependencies(${PROJECT_NAME}
  gz_ros2_control
  hardware_interface
  kdl_parser
  orocos_kdl_vendor
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz_ros2_control
  hardware_interface
  kdl_parser
  orocos_kdl_vendor
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)
pluginlib_export_plugin_description_file(gz_ros2_control panda_gazebo_hardware_plugins.xml)

ament_package()
