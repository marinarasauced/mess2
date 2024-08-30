
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

#include </usr/include/armadillo>
#include </usr/include/opencv4/opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "mess2_msgs/msg/edge.hpp"
#include "mess2_msgs/msg/edge_array.hpp"
#include "mess2_msgs/msg/vertex_array.hpp"
#include "mess2_msgs/msg/threat_element.hpp"
#include "mess2_msgs/msg/threat_element_array.hpp"
#include "mess2_plugins/threat.hpp"

using Edge = mess2_msgs::msg::Edge;
using EdgeArray = mess2_msgs::msg::EdgeArray;
using VertexArray = mess2_msgs::msg::VertexArray;
using ThreatElement = mess2_msgs::msg::ThreatElement;
using ThreatElementArray = mess2_msgs::msg::ThreatElementArray;

class ThreatAlgorithmFrameGenerator : public rclcpp::Node
{
public:
    ThreatAlgorithmFrameGenerator() : Node("algorithm_frame_gen")
    {
        std::string image_file;
        std::string image_path;
        int32_t image_resolution;
        double image_x_min;
        double image_x_max;
        double image_y_min;
        double image_y_max;
        int32_t threat_resolution;
        double threat_x_min;
        double threat_x_max;
        double threat_y_min;
        double threat_y_max;

        this->declare_parameter("image_file", "threat.png");
        this->declare_parameter("image_resolution", 6001);
        this->declare_parameter("image_x_min", -15.0);
        this->declare_parameter("image_x_max",  15.0);
        this->declare_parameter("image_y_min", -15.0);
        this->declare_parameter("image_y_max",  15.0);
        this->declare_parameter("threat_resolution", 301);
        this->declare_parameter("threat_x_min", -3.0);
        this->declare_parameter("threat_x_max",  3.0);
        this->declare_parameter("threat_y_min", -3.0);
        this->declare_parameter("threat_y_max",  3.0);
        this->declare_parameter("grid_line_thickness", 17);

        this->get_parameter("image_file", image_file);
        this->get_parameter("image_resolution", image_resolution);
        this->get_parameter("image_x_min", image_x_min);
        this->get_parameter("image_x_max", image_x_max);
        this->get_parameter("image_y_min", image_y_min);
        this->get_parameter("image_y_max", image_y_max);
        this->get_parameter("threat_resolution", threat_resolution);
        this->get_parameter("threat_x_min", threat_x_min);
        this->get_parameter("threat_x_max", threat_x_max);
        this->get_parameter("threat_y_min", threat_y_min);
        this->get_parameter("threat_y_max", threat_y_max);
        this->get_parameter("grid_line_thickness", grid_line_thickness);

        image_path = ament_index_cpp::get_package_share_directory("mess2_visualizer_cpp") + "/" + image_file;
        image_cv2 = cv::imread(image_path, cv::IMREAD_COLOR);

        auto x_threat = arma::linspace(threat_x_min, threat_x_max, threat_resolution);
        auto y_threat = arma::linspace(threat_y_min, threat_y_max, threat_resolution);
        auto [x_mesh, y_mesh] = mess2_plugins::get_meshgrid(x_threat, y_threat);
        auto threat = mess2_plugins::generate_threat(x_mesh, y_mesh);

        double grid_scale_x = (image_resolution - 1) / (threat_x_max - threat_x_min);
        double grid_scale_y = (image_resolution - 1) / (threat_y_max - threat_y_min);
        grid_points = mess2_plugins::get_vertices(threat, x_mesh, y_mesh);
        int64_t iter = 0;
        for (auto &vertex : grid_points.vertices)
        {
            vertex.position.x = (vertex.position.x - threat_x_min) * grid_scale_x;
            vertex.position.y = (vertex.position.y - threat_y_min) * grid_scale_y;
            ThreatElement grid_point_element;
            grid_point_element.index = iter;
            grid_point_element.color.r = 0.0;
            grid_point_element.color.g = 0.0;
            grid_point_element.color.b = 0.0;
            grid_point_element.color.a = 1.0;
            grid_point_elements.elements.push_back(grid_point_element);
            iter = iter + 1;

            if (iter < 400)
            {
                RCLCPP_INFO(this->get_logger(), "%f, %f", vertex.position.x, vertex.position.y);
            }
        }

        grid_lines = mess2_plugins::get_edges(threat);
        int64_t jter = 0;
        for (auto &edge : grid_lines.edges)
        {
            ThreatElement grid_line_element;
            grid_line_element.index = jter;
            grid_line_element.color.r = 0.0;
            grid_line_element.color.g = 0.0;
            grid_line_element.color.b = 0.0;
            grid_line_element.color.a = 1.0;
            grid_line_elements.elements.push_back(grid_line_element);
            jter = jter + 1;
        }




        (void) draw_edges(grid_point_elements, image_cv2);
        cv::imwrite("test1.png", image_cv2);

        // _edges_subscription = this->create_subscription<EdgeArray>(
        //     "generator/edges",
        //     10,
        //     std::bind(&ThreatAlgorithmFrameGenerator::draw_edges, this, std::placeholders::_1)
        // );
    }
private:
    void new_frame()
    {
        frame_cv2 = image_cv2;
    }



    void draw_edges(const ThreatElementArray msg, cv::Mat _cv2)
    {
        for (auto &element : msg.elements)
        {
            auto edge = grid_lines.edges[element.index];
            cv::Point p1(grid_points.vertices[edge.index1].position.x, grid_points.vertices[edge.index1].position.y);
            cv::Point p2(grid_points.vertices[edge.index2].position.x, grid_points.vertices[edge.index2].position.y);
            auto color = cv::Scalar(element.color.b * 255, element.color.g * 255, element.color.r * 255);

            cv::line(_cv2, p1, p2, color, grid_line_thickness);
        }
    }

    cv::Mat image_cv2;
    cv::Mat frame_cv2;
    EdgeArray grid_lines;
    VertexArray grid_points;
    ThreatElementArray grid_line_elements;
    ThreatElementArray grid_point_elements;
    int32_t grid_line_thickness;
    rclcpp::Subscription<ThreatElement>::SharedPtr _edges_subscription;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::make_shared<ThreatAlgorithmFrameGenerator>();
    rclcpp::shutdown();
    return 0;
}
