cmake_minimum_required(VERSION 3.8)
project(turtlebot3_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_VERSION VERSION_LESS "3.12")
  target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(_turtlebot3_calibration_server SHARED src/server.cpp)
add_library(_turtlebot3_calibration_client SHARED src/client.cpp)

target_include_directories(
  _turtlebot3_calibration_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

target_compile_definitions(
  _turtlebot3_calibration_server
  PRIVATE "TURTLEBOT3_CALIBRATION_SERVER_BUILDING_DLL"
)

target_compile_definitions(
  _turtlebot3_calibration_client
  PRIVATE "TURTLEBOT3_CALIBRATION_CLIENT_BUILDING_DLL"
)

ament_target_dependencies(
  _turtlebot3_calibration_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "mess2_msgs"
  "mess2_plugins"
)

ament_target_dependencies(
  _turtlebot3_calibration_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "mess2_msgs"
  "yaml-cpp"
)

target_link_libraries(_turtlebot3_calibration_client yaml-cpp)

rclcpp_components_register_node(_turtlebot3_calibration_server PLUGIN "mess2_nodes::UAVCalibrationServer" EXECUTABLE server)
rclcpp_components_register_node(_turtlebot3_calibration_client PLUGIN "mess2_nodes::UAVCalibrationClient" EXECUTABLE client)

install(TARGETS
  _turtlebot3_calibration_server
  _turtlebot3_calibration_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
