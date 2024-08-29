
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>

#include </usr/include/armadillo>

#include "geometry_msgs/msg/point.hpp"

#include "mess2_msgs/msg/edge.hpp"
#include "mess2_msgs/msg/edge_array.hpp"
#include "mess2_msgs/msg/vertex.hpp"
#include "mess2_msgs/msg/vertex_array.hpp"
#include "mess2_msgs/msg/threat_field.hpp"

namespace mess2_plugins {

    std::tuple<arma::mat, arma::mat> get_meshgrid(const arma::vec& x_, const arma::vec& y_);

    
    mess2_msgs::msg::VertexArray get_vertices(const arma::mat& threat, const arma::mat& x_mesh, const arma::mat& y_mesh);
    int64_t get_vertex_index(const mess2_msgs::msg::VertexArray vertices, geometry_msgs::msg::Point::SharedPtr point);
    
    mess2_msgs::msg::EdgeArray get_edges(const arma::mat& threat);

    arma::mat generate_threat(const arma::mat& x_mesh, const arma::mat& y_mesh);
    mess2_msgs::msg::ThreatField get_threat_field(const int resolution);

    
}