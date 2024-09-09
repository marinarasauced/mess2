#ifndef MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
#define MESS2_ALGORITHM_PLUGINS_GRAPH_HPP

#include </usr/include/armadillo>

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

#include "mess2_algorithm_msgs/msg/edge.hpp"
#include "mess2_algorithm_msgs/msg/graph.hpp"
#include "mess2_algorithm_msgs/msg/vertex.hpp"

using Edge = mess2_algorithm_msgs::msg::Edge;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Vertex = mess2_algorithm_msgs::msg::Vertex;

namespace mess2_algorithms
{
    std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh);

    std::vector<Edge> generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh);

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh);

    int64_t get_index_vertex_from_position(const std::vector<Vertex>& vertices, const double x, const double y);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
