cmake_minimum_required(VERSION 3.8)
project(fw_mid_mppi)

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xtensor REQUIRED)

add_library(fw_mid_mppi SHARED
  src/cusp_mppi.cpp
  src/path_direction_critic.cpp
)
target_compile_features(fw_mid_mppi PRIVATE cxx_std_17)
target_compile_options(fw_mid_mppi PRIVATE -fconcepts)

ament_target_dependencies(fw_mid_mppi
  nav2_mppi_controller
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  rclcpp_lifecycle
  tf2_geometry_msgs
)
target_link_libraries(fw_mid_mppi xtensor xtensor::use_xsimd)

pluginlib_export_plugin_description_file(nav2_core controller.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

install(TARGETS fw_mid_mppi LIBRARY DESTINATION lib)
ament_package()