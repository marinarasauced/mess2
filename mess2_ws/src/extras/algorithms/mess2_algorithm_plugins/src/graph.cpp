
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/edge.hpp"
#include "mess2_algorithm_plugins/graph.hpp"
#include "mess2_algorithm_plugins/vertex.hpp"

namespace mess2_algorithms
{
    std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        std::vector<Vertex> vertices;
        
        int64_t n_rows = x_mesh.n_rows;
        int64_t n_cols = y_mesh.n_cols;

        for (int64_t iter = 0; iter < 8; ++iter) {
            for (int64_t jter = 0; jter < n_rows; ++jter) {
                for (int64_t kter = 0; kter < n_cols; ++kter) {
                    vertices.emplace_back(Vertex(x_mesh(jter, kter), y_mesh(jter, kter), 45 * iter));
                }
            }
        }

        return vertices;
    }

    std::vector<Edge> generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh, const std::vector<Vertex>& vertices)
    {
        std::vector<Edge> edges;

        int64_t n_rows = x_mesh.n_rows;
        int64_t n_cols = y_mesh.n_cols;

        std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> pairs;
        for (int64_t iter = 0; iter < n_rows; ++iter) {
            for (int64_t jter = 0; jter < n_cols; ++jter) {
                std::pair<double, double> pair_nw = {x_mesh(iter, jter), y_mesh(iter, jter)};
                pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_nw));

                if (jter + 1 < n_cols) {
                    std::pair<double, double> pair_ne = {x_mesh(iter, jter + 1), y_mesh(iter, jter + 1)};
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_ne));
                }

                if (iter + 1 < n_rows) {
                    std::pair<double, double> pair_sw = {x_mesh(iter + 1, jter), y_mesh(iter + 1, jter)};
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_sw));
                }

                if (iter + 1 < n_rows && jter + 1 < n_cols) {
                    std::pair<double, double> pair_ne = {x_mesh(iter, jter + 1), y_mesh(iter, jter + 1)};
                    std::pair<double, double> pair_sw = {x_mesh(iter + 1, jter), y_mesh(iter + 1, jter)};
                    std::pair<double, double> pair_se = {x_mesh(iter + 1, jter + 1), y_mesh(iter + 1, jter + 1)};
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_se));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_ne, pair_sw));
                }
            }
        }

        for (const auto& pair : pairs) {
            const auto vertex_1 = pair.first;
            const auto vertex_2 = pair.second;
            for (int64_t iter = 0; iter < static_cast<int64_t>(vertices.size()); ++iter) {
                const auto vertex_parent = vertices[iter];
                auto vertex_parent_x_ = vertex_parent.get_x_();
                auto vertex_parent_y_ = vertex_parent.get_y_();
                auto vertex_parent_theta_ = vertex_parent.get_theta_();
                bool match_parent_to_1 = (vertex_parent_x_ == vertex_1.first && vertex_parent_y_ == vertex_1.second);
                if (match_parent_to_1) {
                    for (int64_t jter = 0; jter < static_cast<int64_t>(vertices.size()); ++jter) {
                        const auto vertex_child = vertices[jter];
                        auto vertex_child_x_ = vertex_child.get_x_();
                        auto vertex_child_y_ = vertex_child.get_y_();
                        auto vertex_child_theta_ = vertex_child.get_theta_();
                        bool match_child_to_2 = (vertex_child_x_ == vertex_2.first && vertex_child_y_ == vertex_2.second);
                        if (match_child_to_2) {

                            bool is_same_x = (vertex_child_x_ == vertex_parent_x_);
                            bool is_same_y = (vertex_child_y_ == vertex_parent_y_);
                            bool is_same_theta = (vertex_child_theta_ == vertex_parent_theta_);

                            if (is_same_x && is_same_y && is_same_theta) {
                                edges.emplace_back(Edge(iter, jter));
                            } else if (is_same_x && is_same_y && !is_same_theta) {
                                edges.emplace_back(Edge(iter, jter));
                            } else if (is_same_theta) {
                                auto theta_vertices = vertex_parent_theta_;
                                auto theta_true = (180.0 / M_PI) * std::atan2(
                                    vertex_parent_y_ - vertex_child_y_,
                                    vertex_parent_x_ - vertex_child_x_
                                );
                                if (theta_true < 0) {
                                    theta_true += 360;
                                }
                                if (theta_true == theta_vertices) {
                                    edges.emplace_back(Edge(iter, jter));
                                }
                            }
                        }
                    }
                }
            }
        }

        return edges;
    }

    Graph::Graph() {}

    void Graph::fill_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        vertices_ = generate_vertices(x_mesh, y_mesh);
        edges_ = generate_edges(x_mesh, y_mesh, vertices_);
    }

    std::vector<Vertex> Graph::get_vertices_() {
        return vertices_;
    }

    std::vector<Edge> Graph::get_edges_() {
        return edges_;
    }

    void Graph::print_vertices() const {
        std::cout << "new row:" << std::endl;
        for (int64_t iter = 0; iter < static_cast<int64_t>(vertices_.size()); ++iter) {
            const auto& vertex = vertices_[iter];
            std::cout << iter << ": " << vertex.get_x_() << ", " << vertex.get_y_() << ", " << vertex.get_theta_() << std::endl;
        }
    }

    void Graph::print_edges() const {
        for (int64_t iter = 0; iter < static_cast<int64_t>(edges_.size()); ++iter) {
            const auto& edge = edges_[iter];
            std::cout << iter << ": " << edge.get_index_parent_() << " to " << edge.get_index_child_() << std::endl;
        }
    }

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        auto graph = Graph();
        graph.fill_graph(x_mesh, y_mesh);
        return graph;
    }
    

} // namespace mess2_algorithms
