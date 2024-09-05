
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
std::vector<mess2_algorithm_msgs::msg::Vertex> get_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh)
{
    int n_rows = x_mesh.n_rows;
    int n_cols = y_mesh.n_cols;
    std::vector<mess2_algorithm_msgs::msg::Vertex> vertices;
    for (int iter = 0; iter < n_rows; ++iter)
    {
        for (int jter = 0; jter < n_cols; ++jter)
        {
            mess2_algorithm_msgs::msg::Vertex vertex;
            vertex.x = x_mesh(iter, jter);
            vertex.y = y_mesh(iter, jter);
            vertices.push_back(vertex);
        }
    }
    return vertices;
}

std::vector<mess2_algorithm_msgs::msg::Edge> get_edges(const arma::mat& x_mesh, const arma::mat& y_mesh)
{
    int n_rows = x_mesh.n_rows;
    int n_cols = y_mesh.n_cols;
    std::vector<mess2_algorithm_msgs::msg::Edge> edges;
    for (int iter = 0; iter < n_rows; ++iter)
    {
        for (int jter = 0; jter < n_cols; ++jter)
        {
            int64_t source = iter * n_cols + (jter + 1);
            if (jter < n_cols - 1)
            {
                int64_t target = iter * n_cols + (jter + 1);
                mess2_algorithm_msgs::msg::Edge edge;
                edge.source = source;
                edge.target = target;
                edges.push_back(edge);
            }
            if (iter < n_rows - 1)
            {
                int64_t target = (iter + 1) * n_cols + jter;
                mess2_algorithm_msgs::msg::Edge edge;
                edge.source = source;
                edge.target = target;
                edges.push_back(edge);
            }
            if (iter < n_rows - 1 && jter < n_cols - 1)
            {
                int64_t target = (iter + 1) * n_cols + (jter + 1);
                mess2_algorithm_msgs::msg::Edge edge;
                edge.source = source;
                edge.target = target;
                edges.push_back(edge); 
            }
            if (iter < n_rows - 1 && jter > 0)
            {
                int64_t target = (iter + 1) * n_cols + (jter - 1);
                mess2_algorithm_msgs::msg::Edge edge;
                edge.source = source;
                edge.target = target;
                edges.push_back(edge); 
            }
        }
    }
    return edges;
}

mess2_algorithm_msgs::msg::Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
{
    mess2_algorithm_msgs::msg::Graph graph;
    graph.vertices = get_vertices(x_mesh, y_mesh);
    graph.edges = get_edges(x_mesh, y_mesh);
    return graph;
}

int64_t get_index_of_vertex(const std::vector<mess2_algorithm_msgs::msg::Vertex>& vertices, const double x, const double y)
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
