cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mess2_algorithm_plugins REQUIRED)

add_executable(demo_graph src/demo_graph.cpp)

include_directories(${mess2_algorithm_plugins_INCLUDE_DIRS})

ament_target_dependencies(
  demo_graph
  "rclcpp"
  "mess2_algorithm_plugins"
)

install(
  TARGETS demo_graph
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
