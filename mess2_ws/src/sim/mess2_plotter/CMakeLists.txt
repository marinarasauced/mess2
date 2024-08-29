cmake_minimum_required(VERSION 3.8)
project(mess2_plotter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(threat_visualizer src/threat_visualizer.cpp)

target_include_directories(
  threat_visualizer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(
  threat_visualizer
  "rclcpp"
  "cv_bridge"
  "mess2_msgs"
  "mess2_plugins"
  "sensor_msgs"
  "visualization_msgs"
  "ament_index_cpp"
)

target_link_libraries(
  threat_visualizer
  ${PCL_LIBRARIES}
)

install(
  TARGETS threat_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/colormap.csv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
