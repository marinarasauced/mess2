cmake_minimum_required(VERSION 3.8)
project(hawk_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)

add_library(hawk_diagnostics SHARED src/hawk_diagnostics.cpp)

target_include_directories(
  hawk_diagnostics PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

ament_target_dependencies(hawk_diagnostics "rclcpp" "geometry_msgs" "mess2_msgs" "mess2_plugins")

install(
  TARGETS hawk_diagnostics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
