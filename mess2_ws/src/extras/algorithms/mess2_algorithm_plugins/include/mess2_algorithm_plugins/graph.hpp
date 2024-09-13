#ifndef MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
#define MESS2_ALGORITHM_PLUGINS_GRAPH_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/vertex.hpp"
#include "mess2_algorithm_plugins/edge.hpp"
#include </usr/include/armadillo>

namespace mess2_algorithms
{
    std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh);

    std::vector<Edge> generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh, const std::vector<Vertex>& vertices);

    class Graph
    {
    public:
        Graph();

        void fill_graph(const arma::mat& x_mesh, const arma::mat& y_mesh);

        std::vector<Vertex> get_vertices_();
        std::vector<Edge> get_edges_();

        void print_vertices() const;
        void print_edges() const;

    private:
        std::vector<Vertex> vertices_;
        std::vector<Edge> edges_;
    };

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_GRAPH_HPP