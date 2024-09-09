
#include "mess2_algorithm_plugins/graph.hpp"

using Edge = mess2_algorithm_msgs::msg::Edge;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Vertex = mess2_algorithm_msgs::msg::Vertex;

namespace mess2_algorithms
{
   std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        std::vector<Vertex> vertices;

        int n_rows = x_mesh.n_rows;
        int n_cols = y_mesh.n_cols;
        for (int iter = 0; iter < n_rows; ++iter)
        {
            for (int jter = 0; jter < n_cols; ++jter)
            {
                Vertex vertex;
                vertex.x = x_mesh(iter, jter);
                vertex.y = y_mesh(iter, jter);
                vertices.push_back(vertex);
            }
        }
        return vertices;
    }

    std::vector<Edge> generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        std::vector<Edge> edges;
        // only use diagonal elements if the assumption that the actor size >> threat resolution as to prevent collisions on intersecting diagonal edges

        int n_rows = x_mesh.n_rows;
        int n_cols = y_mesh.n_cols;
        for (int iter = 0; iter < n_rows; ++iter)
        {
            for (int jter = 0; jter < n_cols; ++jter)
            {
                int64_t index_parent = iter * n_cols + (jter + 1);
                if (jter < n_cols - 1)
                {
                    int64_t index_child = iter * n_cols + (jter + 1);
                    Edge edge;
                    edge.index_parent = index_parent;
                    edge.index_child = index_child;
                    edges.push_back(edge);
                }
                if (iter < n_rows - 1)
                {
                    int64_t index_child = (iter + 1) * n_cols + jter;
                    Edge edge;
                    edge.index_parent = index_parent;
                    edge.index_child = index_child;
                    edges.push_back(edge);
                }
                // if (iter < n_rows - 1 && jter < n_cols - 1)
                // {
                //     int64_t index_child = (iter + 1) * n_cols + (jter + 1);
                //     Edge edge;
                //     edge.index_parent = index_parent;
                //     edge.index_child = index_child;
                //     edges.push_back(edge); 
                // }
                // if (iter < n_rows - 1 && jter > 0)
                // {
                //     int64_t index_child = (iter + 1) * n_cols + (jter - 1);
                //     Edge edge;
                //     edge.index_parent = index_parent;
                //     edge.index_child = index_child;
                //     edges.push_back(edge); 
                // }
            }
        }
        return edges;
    }

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        Graph graph;
        graph.vertices = generate_vertices(x_mesh, y_mesh);
        graph.edges = generate_edges(x_mesh, y_mesh);
        return graph;
    }

    int64_t get_index_vertex_from_position(const std::vector<Vertex>& vertices, const double x, const double y)
    {
        int64_t index = -1;
        double r_last = std::numeric_limits<double>::max();
        for (std::vector<mess2_algorithm_msgs::msg::Vertex>::size_type iter = 0; iter < vertices.size(); ++iter)
        {
            double r_curr = std::sqrt(
                std::pow(vertices[iter].x - x, 2) + 
                std::pow(vertices[iter].y - y, 2)
            );
            if (r_curr < r_last){
                r_last = r_curr;
                index = static_cast<int64_t>(iter);
            }
        }
        return index;
    }

} // namespace mess2_algorithms
