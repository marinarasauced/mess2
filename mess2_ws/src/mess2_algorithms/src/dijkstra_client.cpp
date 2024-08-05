
// library standard headers
#include <cmath>
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

// library third-party headers
#include </usr/include/armadillo>

// ros2 standard headers
#include "rclcpp/rclcpp.hpp"

// ros2 custom headers and plugins
#include "mess2_msgs/msg/edge.hpp"
#include "mess2_msgs/msg/edge_array.hpp"
#include "mess2_msgs/srv/dijkstra_algo.hpp"
#include "mess2_msgs/msg/dijkstra_out.hpp"

// type aliases
using Edge = mess2_msgs::msg::Edge;
using EdgeArray = mess2_msgs::msg::EdgeArray;
using DijkstraAlgo = mess2_msgs::srv::DijkstraAlgo;
using DijkstraOut = mess2_msgs::msg::DijkstraOut;

// global parameters
double scale = 1.1;
int32_t vertex_init = 7480;
int32_t vertex_trgt = 3785;

double x_min = -15.0;
double x_max = 15.0;
double y_min = -10.0;
double y_max = 10.0;

int32_t x_res = 150;
int32_t y_res = 100;

// get threat field
arma::mat get_threat(const arma::mat& x1, const arma::mat& x2)
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

    // write threat to csv
    /*
    std::ofstream file("threat.csv");
    if (!file.is_open())
    {
        std::cerr << "failed to open" << std::endl;
        return 0;
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
    */

    //
    return threat;
}

// get edges
EdgeArray get_edges_(const arma::mat threat, const arma::mat& x1, const arma::mat& x2)
{
    //
    std::vector<Edge> edges;
    EdgeArray edges_;
    int n_rows = threat.n_rows;
    int n_cols = threat.n_cols;

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
                edge.node1 = vertex_curr;
                edge.node2 = vertex_adj;
                edge.x1[0] = x1[vertex_curr];
                edge.x1[1] = x1[vertex_adj];
                edge.x2[0] = x2[vertex_curr];
                edge.x2[1] = x2[vertex_adj];
                edges.push_back(edge);
            }
            if (iter < n_rows - 1)
            {
                int vertex_adj = (iter + 1) * n_cols + jter;
                Edge edge;
                edge.node1 = vertex_curr;
                edge.node2 = vertex_adj;
                edge.x1[0] = x1[vertex_curr];
                edge.x1[1] = x1[vertex_adj];
                edge.x2[0] = x2[vertex_curr];
                edge.x2[1] = x2[vertex_adj];
                edges.push_back(edge);
            }
            if (iter < n_rows - 1 && jter < n_cols - 1)
            {
                int vertex_adj = (iter + 1) * n_cols + (jter + 1);
                Edge edge;
                edge.node1 = vertex_curr;
                edge.node2 = vertex_adj;
                edge.x1[0] = x1[vertex_curr];
                edge.x1[1] = x1[vertex_adj];
                edge.x2[0] = x2[vertex_curr];
                edge.x2[1] = x2[vertex_adj];
                edges.push_back(edge);   
            }
            if (iter < n_rows - 1 && jter > 0)
            {
                int vertex_adj = (iter + 1) * n_cols + (jter - 1);
                Edge edge;
                edge.node1 = vertex_curr;
                edge.node2 = vertex_adj;
                edge.x1[0] = x1[vertex_curr];
                edge.x1[1] = x1[vertex_adj];
                edge.x2[0] = x2[vertex_curr];
                edge.x2[1] = x2[vertex_adj];
                edges.push_back(edge);   
            }
        }
    }
    edges_.edges = edges;
    return edges_;
}

// format vertices
std::vector<double> get_vertices_(const arma::mat& threat)
{
    //
    int n_rows = threat.n_rows;
    int n_cols = threat.n_cols;
    std::vector<double> vertices_(threat.n_elem);

    //
    for (int iter = 0; iter < n_rows; ++iter)
    {
        for (int jter = 0; jter < n_cols; ++jter)
        {
            int index = iter * n_cols + jter;
            vertices_[index] = threat(iter, jter);
        }
    }
    return vertices_;
}

// meshgrid
std::tuple<arma::mat, arma::mat> meshgrid(const arma::vec& x1, const arma::vec& x2)
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
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("dijkstra_client");


    // create service client
    rclcpp::Client<DijkstraAlgo>::SharedPtr service =
        node->create_client<DijkstraAlgo>("dijkstra_algo");

    //
    arma::vec x1 = arma::linspace(x_min, x_max, x_res);
    arma::vec x2 = arma::linspace(y_min, y_max, y_res);
    auto [x1_, x2_] = meshgrid(x1, x2);
    
    // generate request
    auto threat_ = get_threat(x1_, x2_);
    auto vertices_ = get_vertices_(threat_);

    auto x1__ = get_vertices_(x1_);
    auto x2__ = get_vertices_(x2_);
    auto edges_ = get_edges_(threat_, x1__, x2__);
    int32_t vertex_init_ = vertex_init;
    int32_t vertex_trgt_ = vertex_trgt;

    // send request
    auto request = std::make_shared<DijkstraAlgo::Request>();
    request->threat = vertices_;
    request->edges = edges_;
    request->vertex_init = vertex_init_;
    request->vertex_trgt = vertex_trgt_;

    //
    while (!service->wait_for_service())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "interrupted while waiting for service");
            return 0;
        }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // receive response
    auto future = service->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success");
        auto response = future.get();

        //
        std::cout << "Path: ";
        for (const auto &node : response->result.path)
        {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed");
    }

    //
    rclcpp::shutdown();
    return 0;
}