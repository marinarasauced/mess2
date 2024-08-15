
// library standard headers
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

// library third-party headers
#include </usr/include/armadillo>

// ros2 standard headers
#include "rclcpp/rclcpp.hpp"

// ros2 custom headers and plugins
#include "mess2_msgs/msg/vertex.hpp"
#include "mess2_msgs/msg/vertex_array.hpp"
#include "mess2_msgs/msg/edge.hpp"
#include "mess2_msgs/msg/edge_array.hpp"
#include "mess2_msgs/srv/run_dijkstra.hpp"

// type aliases
using Vertex = mess2_msgs::msg::Vertex;
using VertexArray = mess2_msgs::msg::VertexArray;
using Edge = mess2_msgs::msg::Edge;
using EdgeArray = mess2_msgs::msg::EdgeArray;
using RunDijkstra = mess2_msgs::srv::RunDijkstra;

EdgeArray _get_edges(const arma::mat& threat)
{
    //
    int n_rows = threat.n_rows;
    int n_cols = threat.n_cols;
    EdgeArray edges;

    //
    for (int iter = 0; iter < n_rows; ++iter)
    {
        for (int jter = 0; jter < n_cols; ++jter)
        {
            int vertex_curr = iter * n_cols + jter;
            if (jter < n_cols - 1)
            {
                int vertex_adj = iter * n_cols + (jter + 1);
                Edge edge;
                edge.index1 = vertex_curr;
                edge.index2 = vertex_adj;
                edges.edges.push_back(edge);
            }
            if (iter < n_rows - 1)
            {
                int vertex_adj = (iter + 1) * n_cols + jter;
                Edge edge;
                edge.index1 = vertex_curr;
                edge.index2 = vertex_adj;
                edges.edges.push_back(edge);
            }
            if (iter < n_rows - 1 && jter < n_cols - 1)
            {
                int vertex_adj = (iter + 1) * n_cols + (jter + 1);
                Edge edge;
                edge.index1 = vertex_curr;
                edge.index2 = vertex_adj;
                edges.edges.push_back(edge);   
            }
            if (iter < n_rows - 1 && jter > 0)
            {
                int vertex_adj = (iter + 1) * n_cols + (jter - 1);
                Edge edge;
                edge.index1 = vertex_curr;
                edge.index2 = vertex_adj;
                edges.edges.push_back(edge);   
            }
        }
    }
    return edges;
}

VertexArray _get_vertices(const arma::mat& threat, const arma::mat& x1, const arma::mat& x2)
{
    //
    int n_rows = threat.n_rows;
    int n_cols = threat.n_cols;
    VertexArray vertices;

    //
    for (int iter = 0; iter < n_rows; ++iter)
    {
        for (int jter = 0; jter < n_cols; ++jter)
        {
            Vertex vertex;
            vertex.position.x = x1(iter, jter);
            vertex.position.y = x2(iter, jter);
            vertex.threat = threat(iter, jter);
            vertices.vertices.push_back(vertex);
        }
    }
    return vertices;
}

// save threat field
void _save_threat(const arma::mat& threat, const arma::mat& x1, const arma::mat& x2, const std::string filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "failed to open" << std::endl;
    }
    int n_rows = threat.n_rows;
    int n_cols = threat.n_cols;
    file << "x1,x2,threat\n";
    for (int iter = 0; iter < n_rows; ++iter)
    {
        for (int jter = 0; jter < n_cols; ++jter)
        {
            file << x1(iter, jter) << "," << x2(iter, jter) << "," << threat(iter, jter) << "\n";
        }
    }
    file.close();
}

// get threat field
arma::mat _get_threat(const arma::mat& x1, const arma::mat& x2)
{
    // memory allocation
    arma::mat threat = arma::mat(x1.n_rows, x2.n_cols, arma::fill::zeros);

    // parameters
    int16_t n_peaks = 11;
    arma::mat coeff_peaks = {
        { 0.4218, 1.4253, 0.8271,  1.5330,  1.7610,  0.4533,  0.2392,  0.8364,  0.5060, 1.5190,  2},
        {-4.7996, 6.8744, 1.6560,  4.3881, -3.1295, -9.8145, -6.1511,  0.1478, -9.5152, 1.0008,  9},
        {-4.4763, 3.9248, 7.6271, -9.5064, -3.1759, -1.5719, -8.3998, -8.4129, -8.5525, 8.0068, -1},
        { 3.2579, 1.5239, 1.2908,  2.0099,  2.7261,  2.7449,  2.9398,  2.7439,  0.3691, 3.1097,  3},
        { 0.4039, 0.4382, 2.4844,  1.9652,  1.9238,  1.8567,  0.5470,  1.0401,  0.7011, 3.3193,  3}
    };
    double c1 = 5.0;

    // threat field generation
    for (int iter = 0; iter < n_peaks; ++iter)
    {
        arma::mat c_xym = exp(
            -pow((x1 - coeff_peaks(1, iter)), 2) / (2 * pow(coeff_peaks(3, iter), 2))
            -pow((x2 - coeff_peaks(2, iter)), 2) / (2 * pow(coeff_peaks(4, iter), 2))
        );
        threat += coeff_peaks(0, iter) * c_xym;
    }
    threat = c1 * threat + 1;
    return threat;
}

// meshgrid
std::tuple<arma::mat, arma::mat> _meshgrid(const arma::vec& x1, const arma::vec& x2)
{
    //
    int n_x1 = x1.n_elem;
    int n_x2 = x2.n_elem;
    arma::mat x1_(n_x2, n_x1);
    arma::mat x2_(n_x2, n_x1);

    //
    for (int iter = 0; iter < n_x2; ++iter)
    {
        x1_.row(iter) = x1.t();
        x2_.row(iter).fill(x2(iter));
    }
    return std::make_tuple(x1_, x2_);
}

// main function
int main(int argc, char **argv)
{
    // ros2 node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("dijkstra_ugv_client");

    // declare and retrieve parameters
    double x_min;
    node->declare_parameter("x_min", -15.0);
    node->get_parameter("x_min", x_min);
    
    double x_max;
    node->declare_parameter("x_max", 15.0);
    node->get_parameter("x_max", x_max);
    
    double y_min;
    node->declare_parameter("y_min", -15.0);
    node->get_parameter("y_min", y_min);
    
    double y_max;
    node->declare_parameter("y_max", 15.0);
    node->get_parameter("y_max", y_max);

    int64_t n_cols;
    node->declare_parameter("n_cols", 150);
    node->get_parameter("n_cols", n_cols);

    int64_t n_rows;
    node->declare_parameter("n_rows", 100);
    node->get_parameter("n_rows", n_rows);

    int64_t vertex_init;
    node->declare_parameter("vertex_init", 0);
    node->get_parameter("vertex_init", vertex_init);
    
    int64_t vertex_trgt;
    node->declare_parameter("vertex_trgt", 13000);
    node->get_parameter("vertex_trgt", vertex_trgt);

    std::string filename;
    node->declare_parameter("filename", "helloworld.csv");;
    node->get_parameter("filename", filename);

    // create service client
    rclcpp::Client<RunDijkstra>::SharedPtr client  = node->create_client<RunDijkstra>("dijkstra_ugv");

    // generate request
    arma::vec x1 = arma::linspace(x_min, x_max, n_cols);
    arma::vec x2 = arma::linspace(y_min, y_max, n_rows);
    auto [x1_, x2_] = _meshgrid(x1, x2);
    auto threat_ = _get_threat(x1_, x2_);
    if (filename != "helloworld.csv")
    {
        (void)_save_threat(threat_, x1_, x2_, filename);
    }
    
    auto vertices_ = _get_vertices(threat_, x1_, x2_);
    auto edges_ = _get_edges(threat_);

    auto request = std::make_shared<RunDijkstra::Request>();
    request->threat.vertices = vertices_;
    request->threat.edges = edges_;
    request->vertex_init = vertex_init;
    request->vertex_trgt = vertex_trgt;

    // wait for service
    while (!client->wait_for_service())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for service");
            return 0;
        }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    // send request
    auto future = client->async_send_request(request);

    // receive response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "success");
        auto response = future.get();

        //
        std::cout << "path: ";
        for (const auto &node : response->indices)
        {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "failed");
    }
    
    // shutdown ros
    rclcpp::shutdown();
    return 0;
}