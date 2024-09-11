cmake_minimum_required(VERSION 3.8)
project(mess2_flir_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flir_camera_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(log_to_jpg src/log_to_jpg.cpp)

ament_target_dependencies(
  log_to_jpg
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "flir_camera_msgs"
  "rosbag2_cpp"
  "cv_bridge"
  "OpenCV"
  "yaml-cpp"
)

install(
  TARGETS log_to_jpg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
