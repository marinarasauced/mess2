#ifndef MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
#define MESS2_ALGORITHM_PLUGINS_GRAPH_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include </usr/include/armadillo>

#include "mess2_algorithm_msgs/msg/edge.hpp"
#include "mess2_algorithm_msgs/msg/graph.hpp"
#include "mess2_algorithm_msgs/msg/vertex.hpp"

namespace mess2_algorithms
{
std::vector<mess2_algorithm_msgs::msg::Vertex> get_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh);

std::vector<mess2_algorithm_msgs::msg::Edge> get_edges(const arma::mat& x_mesh, const arma::mat& y_mesh);

mess2_algorithm_msgs::msg::Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh);

int64_t get_index_of_vertex(const std::vector<mess2_algorithm_msgs::msg::Vertex>& vertices, const double x, const double y);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
