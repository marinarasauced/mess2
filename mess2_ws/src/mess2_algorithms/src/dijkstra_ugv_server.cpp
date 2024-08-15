
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
#include "mess2_plugins/orientation.hpp"

// type aliases
using Vertex = mess2_msgs::msg::Vertex;
using VertexArray = mess2_msgs::msg::VertexArray;
using Edge = mess2_msgs::msg::Edge;
using EdgeArray = mess2_msgs::msg::EdgeArray;
using RunDijkstra = mess2_msgs::srv::RunDijkstra;

// get cost of transition
double get_cost_of_exposure(double lambda, Vertex vertex2)
{
    // cost of exposure to threat
    /*
    double dx1 = std::abs(vertex2.x1 - vertex1.x1);
    double dx2 = std::abs(vertex2.x2 - vertex1.x2);
    double scale = std::max(dx1, dx2);
    */
    double scale = 0.01;
    double cost_of_exposure = 0.1 * scale * (lambda + vertex2.threat);
    return cost_of_exposure;
}

// get cost of movement
double get_cost_of_movement(Vertex vertex0, Vertex vertex1, Vertex vertex2)
{
    // translational cost
    double dx1 = std::abs(vertex2.position.x - vertex1.position.x);
    double dx2 = std::abs(vertex2.position.y - vertex1.position.y);
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
RunDijkstra::Response execute_algorithm(const std::vector<Vertex>& vertices, const std::vector<Edge>& edges, int64_t index1, int64_t index2, double lambda, std::shared_ptr<rclcpp::Node> node)
{
    // retrieve parameters
    bool charge_for_movement;
    node->get_parameter("charge_movement", charge_for_movement);

    // generate adjancency matrix
    int64_t n_vertices = static_cast<int64_t>(vertices.size());
    std::vector<std::vector<std::pair<int64_t, double>>> neighbors(n_vertices);
    for (const auto& edge : edges) 
    {
        int64_t index1_ = edge.index1;
        int64_t index2_ = edge.index2;
        neighbors[edge.index1].emplace_back(index2_, get_cost_of_exposure(lambda, vertices[index1_]));
        neighbors[edge.index2].emplace_back(index1_, get_cost_of_exposure(lambda, vertices[index2_]));
    }

    // threat metric and priority queue
    std::vector<double> metrics(n_vertices, std::numeric_limits<double>::infinity());
    std::vector<int64_t> previous(n_vertices, -1);
    std::priority_queue<std::pair<double, int64_t>, std::vector<std::pair<double, int64_t>>, std::greater<>> queue;

    // add starting node to queue
    metrics[index1] = 0;
    queue.emplace(0, index1);

    // run algorithm
    while (!queue.empty())
    {
        // retrieve data from top of queue (minimum metric)
        auto [metric_curr, index1_] = queue.top();
        queue.pop();

        // skip current vertex if threat is greater than previously calculated threat
        if (metric_curr > metrics[index1_])
        {
            continue;
        }

        // process neighbors of current vertex
        for (const auto& [index2_, metric_] : neighbors[index1_])
        {
            auto node0_ = previous[index1_];
            double metric_curr_ = metric_curr + metric_;
            if (charge_for_movement) {
                metric_curr_ += get_cost_of_movement(vertices[node0_], vertices[index1_], vertices[index2_]);
            }
            if (metric_curr_ < metrics[index2_]) {
                metrics[index2_] = metric_curr_;
                previous[index2_] = index1_;
                queue.emplace(metric_curr_, index2_);
            }
        }
    }

    // retrieve path
    std::vector<int64_t> path;
    for (int64_t last = index2; last != -1; last = previous[last])
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
        double dx1_10 = vertex1.position.x - vertex0.position.x;
        double dx2_10 = vertex1.position.y - vertex0.position.y;
        double dx1_21 = vertex2.position.x - vertex1.position.x;
        double dx2_21 = vertex2.position.y - vertex1.position.y;

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
    RunDijkstra::Response result;
    result.indices = path_;
    return result;
}

// execute service
void execute_service(const std::shared_ptr<RunDijkstra::Request> request, const std::shared_ptr<RunDijkstra::Response> response,  std::shared_ptr<rclcpp::Node> node)
{
    // get parameters
    double lambda;
    node->get_parameter("lambda", lambda);

    // handle request
    RCLCPP_INFO(node->get_logger(), "received request, running dijkstra's algorithm");
    std::vector<Vertex> vertices(request->threat.vertices.vertices.begin(), request->threat.vertices.vertices.end());
    std::vector<Edge> edges(request->threat.edges.edges.begin(), request->threat.edges.edges.end());
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
    rclcpp::Service<RunDijkstra>::SharedPtr server = node->create_service<RunDijkstra>("dijkstra_ugv", [node](const std::shared_ptr<RunDijkstra::Request> request, const std::shared_ptr<RunDijkstra::Response> response) {
        execute_service(request, response, node);
    });
    RCLCPP_INFO(node->get_logger(), "ready to run dijkstra's algorithm");
    rclcpp::spin(node);

    // ros2 shutdown
    rclcpp::shutdown();
    return 0;
}