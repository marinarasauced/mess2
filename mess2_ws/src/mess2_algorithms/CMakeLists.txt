cmake_minimum_required(VERSION 3.8)
project(mess2_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mess2_msgs REQUIRED)

add_executable(dijkstra_server src/dijkstra_server.cpp)
ament_target_dependencies(dijkstra_server rclcpp mess2_msgs)

install(TARGETS
    dijkstra_server
  DESTINATION lib/${PROJECT_NAME})


ament_package()
