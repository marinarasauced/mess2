cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Edge.msg"
  "msg/Graph.msg"
  "msg/Occupancy.msg"
  "msg/ThreatField.msg"
  "msg/Vertex.msg"
  DEPENDENCIES
)

ament_package()
