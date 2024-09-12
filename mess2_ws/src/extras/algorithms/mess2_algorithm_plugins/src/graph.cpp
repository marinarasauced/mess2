
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

        for (const auto pair : pairs) {
            const auto vertex_1 = pair.first;
            const auto vertex_2 = pair.second;
            for (int64_t iter = 0; iter < vertices.size(); ++iter) {
                const auto vertex_parent = vertices[iter];
                bool match_parent_to_1 = (vertex_parent.x_ == vertex_1.first && vertex_parent.y_ == vertex_1.second);
                if (match_parent_to_1) {
                    for (int64_t jter = 0; jter < vertices.size(); ++jter) {
                        const auto vertex_child = vertices[jter];
                        bool match_child_to_2 = (vertex_child.x_ == vertex_2.first && vertex_child.y_ == vertex_2.second);
                        if (match_child_to_2) {

                            bool is_same_x = (vertex_child.x_ == vertex_parent.x_);
                            bool is_same_y = (vertex_child.y_ == vertex_parent.y_);
                            bool is_same_theta = (vertex_child.theta_ == vertex_parent.theta_);

                            if (is_same_x && is_same_y && is_same_theta) {
                                edges.emplace_back(Edge(iter, jter));
                            } else if (is_same_x && is_same_y && !is_same_theta) {
                                edges.emplace_back(Edge(iter, jter));
                            } else if (is_same_theta) {
                                auto theta_vertices = vertex_parent.theta_;
                                auto theta_true = (180.0 / M_PI) * std::atan2(
                                    vertex_parent.y_ - vertex_child.y_,
                                    vertex_parent.x_ - vertex_child.x_
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




        // 

        // auto calculate_theta = [](double x1, double y1, double x2, double y2) {
        //     return std::atan2(y2 - y1, x2 - x1) * 180 / M_PI;
        // };

        // int64_t n_rows = x_mesh.n_rows;
        // int64_t n_cols = y_mesh.n_cols;
        // int64_t n_elem = vertices.size();
        // for (int64_t iter = 0; iter < n_rows; ++iter) {
        //     for (int64_t jter = 0; jter < n_cols; ++jter) {
        //         auto x_sw = x_mesh(iter + 0, jter + 0);
        //         auto y_sw = y_mesh(iter + 0, jter + 0);
        //         auto x_se = x_mesh(iter + 1, jter + 0);
        //         auto y_se = y_mesh(iter + 1, jter + 0);
        //         auto x_ne = x_mesh(iter + 1, jter + 1);
        //         auto y_ne = y_mesh(iter + 1, jter + 1);
        //         auto x_nw = x_mesh(iter + 0, jter + 1);
        //         auto y_nw = y_mesh(iter + 0, jter + 1);

        //         for (int64_t kter1 = 0; kter1 < vertices.size(); ++kter1) {
        //             auto vertex_parent = vertices[kter1];
        //             bool parent_is_sw = (vertex_parent.x_ == x_sw && vertex_parent.y_ == y_sw);
        //             bool parent_is_se = (vertex_parent.x_ == x_se && vertex_parent.y_ == y_se);
        //             bool parent_is_ne = (vertex_parent.x_ == x_ne && vertex_parent.y_ == y_sw);
        //             bool parent_is_nw = (vertex_parent.x_ == x_nw && vertex_parent.y_ == y_sw);
                    
        //             for (int64_t kter2 = 0; kter2 < vertices.size(); ++kter2) {
        //                 auto vertex_child = vertices[kter2];

                        

        //                 bool same_x = (vertex_child.x_ == vertex_parent.x_);
        //                 bool same_y = (vertex_child.y_ == vertex_parent.y_);
        //                 bool same_theta = (vertex_child.theta_ == vertex_parent.theta_);

        //                 // if position and heading are same, add edge for waiting at vertex
        //                 if (same_x && same_y && same_theta) {
        //                     edges.emplace_back(Edge(kter1, kter2));
        //                 }

        //                 // if position is same but heading is different, add edge for rotating at vertex
        //                 if (same_x && same_y && !same_theta) {
        //                     edges.emplace_back(Edge(kter1, kter2));
        //                 }
        //             }
        //         }
        //     }
        // }






        // for (int64_t iter = 0; iter < 8; ++iter) {
        //     for (int64_t jter = 0; jter < n_rows; ++jter) {
        //         for (int64_t kter = 0; kter < n_cols; ++kter) {
                    
                    
        //             auto vertex = vertices[kter];
        //             if (x_parent == vertex.x_ && y_parent == vertex.y_) {
        //                 if (jter < n_cols - 1) {

        //                 }
        //             }
        //         }


        //         int64_t index_parent = iter * n_cols + jter;
        //         if (jter < n_cols - 1) {
        //             int64_t index_child = iter * n_cols + (jter + 1);
        //             edges.emplace_back(Edge(index_parent, index_child));
        //         }
        //         if (iter < n_rows - 1) {
        //             int64_t index_child = (iter + 1) * n_cols + jter;
        //             edges.emplace_back(Edge(index_parent, index_child));
        //         }
        //     }
        // }
    }

} // namespace mess2_algorithms
