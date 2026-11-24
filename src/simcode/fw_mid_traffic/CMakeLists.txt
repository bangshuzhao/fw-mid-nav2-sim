cmake_minimum_required(VERSION 3.16)
project(fw_mid_traffic)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(fw_mid_crossings SHARED src/crossing.cpp)
target_compile_features(fw_mid_crossings PRIVATE cxx_std_17)
target_link_libraries(fw_mid_crossings PRIVATE ignition-gazebo6::core ignition-plugin1::register)
install(TARGETS fw_mid_crossings LIBRARY DESTINATION lib)

ament_package()
