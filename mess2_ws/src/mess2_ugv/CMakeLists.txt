cmake_minimum_required(VERSION 3.5)
project(mess2_ugv)

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
    action_server SHARED
    src/set_ugv_state_action_server.cpp
)
add_library(
    action_client SHARED
    src/set_ugv_state_action_client.cpp
)
target_include_directories(
    action_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)
target_include_directories(
    action_client PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)
target_compile_definitions(
    action_server
    PRIVATE "SET_UGV_STATE_ACTION_SERVER_BUILDING_DLL"
)
target_compile_definitions(
    action_client
    PRIVATE "SET_UGV_STATE_ACTION_CLIENT_BUILDING_DLL"
)
ament_target_dependencies(action_server
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "geometry_msgs"
    "mess2_msgs"
    "mess2_plugins"
)
ament_target_dependencies(action_client
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "geometry_msgs"
    "mess2_msgs"
    "mess2_plugins"
)
rclcpp_components_register_node(action_server PLUGIN "set_ugv_state_action::SetUGVStateActionServer" EXECUTABLE set_ugv_state_action_server)
rclcpp_components_register_node(action_client PLUGIN "set_ugv_state_action::SetUGVStateActionClient" EXECUTABLE set_ugv_state_action_client)
install(TARGETS
    action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS
    action_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_package()
