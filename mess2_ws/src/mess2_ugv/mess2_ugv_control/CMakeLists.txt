cmake_minimum_required(VERSION 3.5)
project(mess2_ugv_control)

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

add_library(
    line_following_action_server SHARED
    src/follow_line_action_server.cpp
)
add_library(
    line_following_action_client SHARED
    src/follow_line_action_client.cpp
)
target_include_directories(
    line_following_action_server PRIVATE
    line_following_action_client PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)
target_compile_definitions(
    line_following_action_server
    PRIVATE "UGV_LINE_FOLLOWING_ACTION_SERVER_BUILDING_DLL"
)
target_compile_definitions(
    line_following_action_client
    PRIVATE "UGV_LINE_FOLLOWING_ACTION_CLIENT_BUILDING_DLL"
)
ament_target_dependencies(line_following_action_server
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "geometry_msgs"
    "mess2_msgs"
    "mess2_plugins"
)
ament_target_dependencies(line_following_action_client
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "geometry_msgs"
    "mess2_msgs"
    "mess2_plugins"
)
rclcpp_components_register_node(line_following_action_server PLUGIN "ugv_actions::UGVFollowLineActionServer" EXECUTABLE follow_line_action_server)
rclcpp_components_register_node(line_following_action_client PLUGIN "ugv_actions::UGVFollowLineActionClient" EXECUTABLE follow_line_action_client)
install(TARGETS line_following_action_server line_following_action_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_package()
