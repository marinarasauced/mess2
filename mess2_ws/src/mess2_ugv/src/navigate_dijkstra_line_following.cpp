
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

// ros2 standard headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ros2 custom headers and plugins
#include "mess2_msgs/msg/vertex.hpp"
#include "mess2_msgs/msg/vertex_array.hpp"
#include "mess2_msgs/msg/edge.hpp"
#include "mess2_msgs/msg/edge_array.hpp"
#include "mess2_msgs/srv/dijkstra_algo.hpp"
#include "mess2_msgs/action/ugv_follow_line.hpp"

// type aliases
using Vertex = mess2_msgs::msg::Vertex;
using VertexArray = mess2_msgs::msg::VertexArray;
using Edge = mess2_msgs::msg::Edge;
using EdgeArray = mess2_msgs::msg::EdgeArray;
using DijkstraAlgo = mess2_msgs::srv::DijkstraAlgo;
using FollowLine = mess2_msgs::action::UGVFollowLine;

// path planning
DijkstraAlgo::Response call_service_dijkstra(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<DijkstraAlgo>::SharedPtr client, const VertexArray vertices, const EdgeArray edges, const int32_t vertex_init, const int32_t vertex_trgt)
{
    // wait for service
    while (!client->wait_for_service())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for service");
            rclcpp::shutdown();
        }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    // generate and send request
    auto request = std::make_shared<DijkstraAlgo::Request>();
    request->vertices = vertices;
    request->edges = edges;
    request->vertex_init = vertex_init;
    request->vertex_trgt = vertex_trgt;
    auto future = client->async_send_request(request);

    // receive response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // extract response
        RCLCPP_INFO(node->get_logger(), "success");
        auto response = future.get();

        // generate return
        DijkstraAlgo::Response result;
        result.indices = response->indices;
        return result;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "failed");
        rclcpp::shutdown();
    }
}

// navigation
void call_action_line_following(std::shared_ptr<rclcpp::Node> node, rclcpp_action::Client<FollowLine>::SharedPtr client, const std::vector<int64_t> path, const std::vector<Vertex> vertices)
{
    // wait for action
    while (!client->wait_for_action_server())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for action server");
            rclcpp::shutdown();
        }
    RCLCPP_INFO(node->get_logger(), "action server not available, waiting again...");
    }

    // call action for each vertex in path
    for (int iter = 0; iter < path.size(); ++iter)
    {
        // generate goal
        auto goal = std::make_shared<FollowLine::Goal>();
        goal->trgt_x = vertices[iter].x1;
        goal->trgt_y = vertices[iter].x2;

        // send goal
        RCLCPP_INFO(node->get_logger(), "sending goal");
        auto send_goal_options = rclcpp_action::Client<FollowLine>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = [node](const typename rclcpp_action::Client<FollowLine>::GoalHandle::SharedPtr & goal_handle)
        {
            if (!goal_handle) 
            {
                RCLCPP_ERROR(node->get_logger(), "goal was rejected by server");
            } else 
            {
                RCLCPP_INFO(node->get_logger(), "goal accepted by server, waiting for result");
            }
        };

        send_goal_options.result_callback = [node](const typename rclcpp_action::Client<FollowLine>::GoalHandle::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node->get_logger(), "goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(node->get_logger(), "goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(node->get_logger(), "unknown result code");
                    return;
            }
            std::stringstream ss;
            ss << "result received: success = " << std::boolalpha << result.result->success;
            RCLCPP_INFO(node->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        };
    }
}

// main function
int main(int argc, char **argv)
{
    // ros2 node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ugv_dijkstra_line_following");

    // create dijkstra service client
    rclcpp::Client<DijkstraAlgo>::SharedPtr client_dijkstra = node->create_client<DijkstraAlgo>("dijkstra_algo");

    // create line following action client for each ugv
    rclcpp_action::Client<FollowLine>::SharedPtr client_follow_line_burger1 = rclcpp_action::create_client<FollowLine>(node, "ugv/burger1/follow_line");

    // generate fake threat field data
    VertexArray vertices;
    EdgeArray edges;
    int32_t vertex_init;
    int32_t vertex_trgt;

    // call dijkstra service to generate path
    auto pathplan = call_service_dijkstra(node, client_dijkstra, vertices, edges, vertex_init, vertex_trgt);
    auto path = pathplan.indices;

    // create threads for each ugv to call line following
    std::vector<std::thread> threads;
    threads.emplace_back(call_action_line_following, node, client_follow_line_burger1, path, vertices);

    // join threads
    for (auto& thread : threads) {
        thread.join();
    }

    // ros2 shutdown
    rclcpp::shutdown();
    return 0;
}