cmake_minimum_required(VERSION 3.8)
project(hawk_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)


add_library(_hawk_calibration_server SHARED src/server.cpp)

target_include_directories(
  _hawk_calibration_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

target_compile_definitions(
  _hawk_calibration_server
  PRIVATE "HAWK_CALIBRATION_SERVER_BUILDING_DLL"
)

ament_target_dependencies(
  _hawk_calibration_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "mess2_msgs"
  "mess2_plugins"
)

rclcpp_components_register_node(_hawk_calibration_server PLUGIN "mess2_actions::UAVCalibrationServer" EXECUTABLE server)

install(TARGETS
  _hawk_calibration_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
