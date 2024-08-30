cmake_minimum_required(VERSION 3.8)
project(turtlebot3_time_invariant_control)

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
find_package(sensor_msgs REQUIRED)

add_library(_turtlebot3_face_time_invariant_heading_server SHARED src/face_time_invariant_heaving_server.cpp)
add_library(_turtlebot3_follow_time_invariant_line_server SHARED src/follow_time_invariant_line_server.cpp)

target_include_directories(
  _turtlebot3_face_time_invariant_heading_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

target_include_directories(
  _turtlebot3_follow_time_invariant_line_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

target_compile_definitions(
  _turtlebot3_face_time_invariant_heading_server
  PRIVATE "TURTLEBOT3_FACE_TIME_INVARIANT_HEADING_SERVER_BUILDING_DLL"
)

target_compile_definitions(
  _turtlebot3_follow_time_invariant_line_server
  PRIVATE "TURTLEBOT3_FOLLOW_TIME_INVARIANT_LINE_SERVER_BUILDING_DLL"
)

ament_target_dependencies(
  _turtlebot3_face_time_invariant_heading_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "mess2_msgs"
  "mess2_plugins"
)

ament_target_dependencies(
  _turtlebot3_follow_time_invariant_line_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "mess2_msgs"
  "mess2_plugins"
)

rclcpp_components_register_node(_turtlebot3_face_time_invariant_heading_server PLUGIN "mess2_nodes::UGVTimeInvariantHeadingFacingServer" EXECUTABLE follow_time_invariant_line_server)

rclcpp_components_register_node(_turtlebot3_follow_time_invariant_line_server PLUGIN "mess2_nodes::UGVTimeInvariantLineFollowingServer" EXECUTABLE face_time_invariant_heading_server)


install(TARGETS
  _turtlebot3_face_time_invariant_heading_server
  _turtlebot3_follow_time_invariant_line_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
