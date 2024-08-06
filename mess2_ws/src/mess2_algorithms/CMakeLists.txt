cmake_minimum_required(VERSION 3.8)
project(mess2_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)

add_executable(dijkstra_ugv_server src/dijkstra_ugv_server.cpp)
add_executable(dijkstra_ugv_client src/dijkstra_ugv_client.cpp)

target_include_directories(
  dijkstra_ugv_server PRIVATE
  dijkstra_ugv_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

ament_target_dependencies(dijkstra_ugv_server rclcpp mess2_msgs mess2_plugins)
ament_target_dependencies(dijkstra_ugv_client rclcpp mess2_msgs)

install(
  TARGETS dijkstra_ugv_server dijkstra_ugv_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
