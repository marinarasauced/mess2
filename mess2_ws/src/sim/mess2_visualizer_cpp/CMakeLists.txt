cmake_minimum_required(VERSION 3.8)
project(mess2_visualizer_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(generate_threat_gif src/generate_threat_gif.cpp)
add_executable(generate_threat_png src/generate_threat_png.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(
  generate_threat_gif PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

target_include_directories(
  generate_threat_png PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

ament_target_dependencies(
  generate_threat_gif
  "rclcpp"
  "mess2_msgs"
  "mess2_plugins"
  "sensor_msgs"
  "std_msgs"
  "ament_index_cpp"
  "OpenCV"
)

ament_target_dependencies(
  generate_threat_png
  "rclcpp"
  "mess2_msgs"
  "mess2_plugins"
  "sensor_msgs"
  "ament_index_cpp"
  "OpenCV"
)

install(
  TARGETS generate_threat_gif generate_threat_png
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/parula.csv src/threat.png src/config.rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
