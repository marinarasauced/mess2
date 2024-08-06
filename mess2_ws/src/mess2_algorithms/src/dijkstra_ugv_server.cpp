
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
#include "mess2_msgs/srv/dijkstra_algo.hpp"
#include "mess2_plugins/angles.hpp"

// type aliases
using Vertex = mess2_msgs::msg::Vertex;
using VertexArray = mess2_msgs::msg::VertexArray;
using Edge = mess2_msgs::msg::Edge;
using EdgeArray = mess2_msgs::msg::EdgeArray;
using DijkstraAlgo = mess2_msgs::srv::DijkstraAlgo;

// get cost of transition
double get_cost_of_exposure(double lambda, Vertex vertex1, Vertex vertex2)
{
    // cost of exposure to threat
    double dx1 = std::abs(vertex2.x1 - vertex1.x1);
    double dx2 = std::abs(vertex2.x2 - vertex1.x2);
    double scale = std::max(dx1, dx2);
    double cost_of_exposure = 0.1 * scale * (lambda + vertex2.threat);
    return cost_of_exposure;
}

// get cost of movement
double get_cost_of_movement(Vertex vertex0, Vertex vertex1, Vertex vertex2)
{
    // translational cost
    double dx1 = std::abs(vertex2.x1 - vertex1.x1);
    double dx2 = std::abs(vertex2.x2 - vertex1.x2);
    double scale = std::max(dx1, dx2);
    double cost_of_translation = scale * std::sqrt(dx1 * dx1 + dx2 * dx2);

    // rotational cost
    double dtheta = mess2_plugins::get_angle_between_edges(vertex0, vertex1, vertex2);
    double cost_of_rotation = scale * (dtheta / M_PI);

    // cost
    double cost = cost_of_translation + cost_of_rotation;
    return cost;
}

// execute algorithm
DijkstraAlgo::Response execute_algorithm(const std::vector<Vertex>& vertices, const std::vector<Edge>& edges, int32_t node1, int32_t node2, double lambda, std::shared_ptr<rclcpp::Node> node)
{
    // retrieve parameters
    bool charge_for_movement;
    node->get_parameter("charge_movement", charge_for_movement);

    // generate adjancency matrix
    int32_t n_vertices = static_cast<int32_t>(vertices.size());
    std::vector<std::vector<std::pair<int32_t, double>>> neighbors(n_vertices);
    for (const auto& edge : edges) 
    {
        int32_t node1_ = edge.node1;
        int32_t node2_ = edge.node2;
        neighbors[edge.node1].emplace_back(node2_, get_cost_of_exposure(lambda, vertices[node2_], vertices[node1_]));
        neighbors[edge.node2].emplace_back(node2_, get_cost_of_exposure(lambda, vertices[node1_], vertices[node2_]));
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
            auto node0_ = previous[node1_];
            double metric_curr_ = metric_curr + metric_;
            if (charge_for_movement) {
                metric_curr_ += get_cost_of_movement(vertices[node0_], vertices[node1_], vertices[node2_]);
            }
            if (metric_curr_ < metrics[node2_]) {
                metrics[node2_] = metric_curr_;
                previous[node2_] = node1_;
                queue.emplace(metric_curr_, node2_);
            }
        }
    }

    // retrieve path
    std::vector<int64_t> path;
    for (int64_t last = node2; last != -1; last = previous[last])
    {
        path.push_back(last);
    }
    std::reverse(path.begin(), path.end());

    // eliminate intermediate points in path
    std::vector<int64_t> path_;
    path_.push_back(path[0]);
    for (std::vector<int>::size_type iter = 1; iter < path.size() - 1; ++iter)
    {
        //
        Vertex vertex0 = vertices[path[iter - 1]];
        Vertex vertex1 = vertices[path[iter]];
        Vertex vertex2 = vertices[path[iter + 1]];

        //
        double dx1_10 = vertex1.x1 - vertex0.x1;
        double dx2_10 = vertex1.x2 - vertex0.x2;
        double dx1_21 = vertex2.x1 - vertex1.x1;
        double dx2_21 = vertex2.x2 - vertex1.x2;

        //
        double dx1_ = dx1_21 - dx1_10;
        double dx2_ = dx2_21 - dx2_10;

        //
        if (std::abs(dx1_) < 0.0001 && std::abs(dx2_) < 0.0001) {
            continue;
        } else {
            path_.push_back(path[iter]);
        }
    }
    path_.push_back(path[path.size() - 1]);

    // populate and return result
    DijkstraAlgo::Response result;
    result.indices = path_;
    return result;
}

// execute service
void execute_service(const std::shared_ptr<DijkstraAlgo::Request> request, const std::shared_ptr<DijkstraAlgo::Response> response,  std::shared_ptr<rclcpp::Node> node)
{
    // get parameters
    double lambda;
    node->get_parameter("lambda", lambda);

    // handle request
    RCLCPP_INFO(node->get_logger(), "received request, running dijkstra's algorithm");
    std::vector<Vertex> vertices(request->vertices.vertices.begin(), request->vertices.vertices.end());
    std::vector<Edge> edges(request->edges.edges.begin(), request->edges.edges.end());
    auto result = execute_algorithm(vertices, edges, request->vertex_init, request->vertex_trgt, lambda, node);
    
    // handle response
    response->indices = result.indices;
    RCLCPP_INFO(node->get_logger(), "sent response");
}

// main function
int main(int argc, char **argv)
{
    // ros2 node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("dijkstra_ugv_server");

    // declare parameters
    node->declare_parameter("lambda", 0.0);
    node->declare_parameter("charge_movement", false);

    // create service server and spin
    rclcpp::Service<DijkstraAlgo>::SharedPtr server = node->create_service<DijkstraAlgo>("dijkstra_ugv", [node](const std::shared_ptr<DijkstraAlgo::Request> request, const std::shared_ptr<DijkstraAlgo::Response> response) {
        execute_service(request, response, node);
    });
    RCLCPP_INFO(node->get_logger(), "ready to run dijkstra's algorithm");
    rclcpp::spin(node);

    // ros2 shutdown
    rclcpp::shutdown();
    return 0;
}