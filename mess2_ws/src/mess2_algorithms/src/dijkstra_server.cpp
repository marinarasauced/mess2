
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
double lambda = 0.0;

// edge cost
double cost(double lambda, double metric)
{
    //
    double cost_ = 0.1 * (lambda + metric);
    return cost_;
}

// execute algorithm
DijkstraOut execute_algorithm(const std::vector<double>& threat, const std::vector<Edge>& edges, int32_t node1, int32_t node2)
{
    // algorithm output
    DijkstraOut result;

    // vertex adjacency
    int32_t n_vertices = static_cast<int32_t>(threat.size());
    std::vector<std::vector<std::pair<int32_t, double>>> neighbors(n_vertices);
    for (const auto& edge : edges) 
    {
        int32_t node1_ = edge.node1;
        int32_t node2_ = edge.node2;
        neighbors[edge.node1].emplace_back(node2_, cost(lambda, threat[node2_]));
        neighbors[edge.node2].emplace_back(node1_, cost(lambda, threat[node1_]));
    }

    // threat metric and priority queue
    std::vector<double> metrics(n_vertices, std::numeric_limits<double>::infinity());
    std::vector<int32_t> previous(n_vertices, -1);
    std::priority_queue<std::pair<double, int32_t>, std::vector<std::pair<double, int32_t>>, std::greater<>> queue;

    // add starting node to queue
    metrics[node1] = 0;
    queue.emplace(0, node1);

    // run algorithm
    while (!queue.empty())
    {
        // retrieve data from top of queue (minimum metric)
        auto [metric_curr, node1_] = queue.top();
        queue.pop();

        // skip current vertex if threat is greater than previously calculated threat
        if (metric_curr > metrics[node1_])
        {
            continue;
        }

        // process neighbors of current vertex
        for (const auto& [node2_, metric_] : neighbors[node1_])
        {
            double metric_curr_ = metric_curr + metric_;
            if (metric_curr_ < metrics[node2_])
            {
                metrics[node2_] = metric_curr_;
                previous[node2_] = node1_;
                queue.emplace(metric_curr_, node2_);
            }
        }
    }

    // retrieve path
    std::vector<int32_t> path;
    for (int32_t last = node2; last != -1; last = previous[last])
    {
        path.push_back(last);
    }
    std::reverse(path.begin(), path.end());

    // populate and return result
    result.path = path;
    result.metric = metrics[node2];
    return result;
}

// execute service
void execute_service(const std::shared_ptr<DijkstraAlgo::Request> request, std::shared_ptr<DijkstraAlgo::Response> response)
{
    // run service
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "running dijkstra's algorithm");
    std::vector<Edge> edges(request->edges.edges.begin(), request->edges.edges.end());
    auto result = execute_algorithm(request->threat, edges, request->vertex_init, request->vertex_trgt);

    // generate response
    response->result.path = result.path;
    response->result.metric = result.metric;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "path found with exposure metric: %f", result.metric);
}

// main function
int main(int argc, char **argv)
{
    // ros2 node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("dijkstra_server");

    // create service server
    rclcpp::Service<DijkstraAlgo>::SharedPtr service =
        node->create_service<DijkstraAlgo>("dijkstra_algo", &execute_service);
    
    //
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ready to run dijkstra's algorithm");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
