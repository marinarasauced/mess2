cmake_minimum_required(VERSION 3.8)
project(mess2_algorithm_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(_graph SHARED src/graph/_graph.cpp)
add_library(graph_adjacency SHARED src/graph/adjacency.cpp)
add_library(graph_edge SHARED src/graph/edge.cpp)
add_library(graph_vertex SHARED src/graph/vertex.cpp)

target_link_libraries(_graph graph_adjacency)
target_link_libraries(_graph graph_edge)
target_link_libraries(_graph graph_vertex)





# add_library(__low_level SHARED src/_low_level.cpp)

# add_library(_edge SHARED src/edge.cpp)
# add_library(_graph SHARED src/graph.cpp)
# add_library(_utils SHARED src/utils.cpp)
# add_library(_vertex SHARED src/vertex.cpp)



# add_library(_actor SHARED src/actor.cpp)
# add_library(_cost SHARED src/cost.cpp)
# add_library(_graph SHARED src/graph.cpp)
# add_library(_high_level SHARED src/high_level.cpp)
# add_library(_low_level SHARED src/low_level.cpp)
# add_library(_threat SHARED src/threat.cpp)
# add_library(_utils SHARED src/utils.cpp)

# ament_target_dependencies(
#   _actor
#   "mess2_algorithm_msgs"
#   "yaml-cpp"
# )

# ament_target_dependencies(
#   _cost
#   "mess2_algorithm_msgs"
# )

# ament_target_dependencies(
#   _graph
#   "mess2_algorithm_msgs"
# )

# ament_target_dependencies(
#   _high_level
#   "mess2_algorithm_msgs"
# )

# ament_target_dependencies(
#   _low_level
#   "mess2_algorithm_msgs"
# )

# ament_target_dependencies(
#   _threat
#   "mess2_algorithm_msgs"
# )

# target_link_libraries(_actor yaml-cpp)
# target_link_libraries(_low_level _cost)
# target_link_libraries(_high_level _low_level)

install(
  TARGETS _graph graph_adjacency graph_edge graph_vertex
  EXPORT export_mess2_algorithm_plugins
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_mess2_algorithm_plugins)
ament_package()
