cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mess2_algorithm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(_actor SHARED src/actor.cpp)
add_library(_cost SHARED src/cost.cpp)
add_library(_graph SHARED src/graph.cpp)
add_library(_threat SHARED src/threat.cpp)
add_library(_utils SHARED src/utils.cpp)
add_library(_low_level_search SHARED src/low_level/search.cpp)

ament_target_dependencies(
  _actor
  "mess2_algorithm_msgs"
  "yaml-cpp"
)

ament_target_dependencies(
  _cost
  "mess2_algorithm_msgs"
)

ament_target_dependencies(
  _graph
  "mess2_algorithm_msgs"
)

ament_target_dependencies(
  _threat
  "mess2_algorithm_msgs"
)

ament_target_dependencies(
  _low_level_search
  "mess2_algorithm_msgs"
)

target_link_libraries(_actor yaml-cpp)

install(
  TARGETS _actor _cost _graph _threat _utils _low_level_search
  EXPORT export_mess2_algorithm_plugins
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_mess2_algorithm_plugins)
ament_package()
