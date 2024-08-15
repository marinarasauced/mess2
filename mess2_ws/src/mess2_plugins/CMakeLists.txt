cmake_minimum_required(VERSION 3.8)
project(mess2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)

include_directories(include)

add_library(orientation SHARED src/orientation.cpp)
ament_target_dependencies(orientation rclcpp geometry_msgs mess2_msgs)

install(
  TARGETS orientation
  EXPORT export_mess2_plugins
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_mess2_plugins
)

ament_package()
