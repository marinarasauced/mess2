
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include </usr/include/armadillo>
#include </usr/include/opencv4/opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "mess2_msgs/msg/edge_array.hpp"
#include "mess2_msgs/msg/vertex_array.hpp"
#include "mess2_msgs/msg/threat_element_array.hpp"
#include "mess2_msgs/srv/threat_draw_frame.hpp"
#include "mess2_plugins/threat.hpp"

using EdgeArray = mess2_msgs::msg::EdgeArray;
using VertexArray = mess2_msgs::msg::VertexArray;
using ThreatElementArray = mess2_msgs::msg::ThreatElementArray;
using ThreatDrawFrame = mess2_msgs::srv::ThreatDrawFrame;

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
        this->declare_parameter("edge_thickness", 1);
        this->declare_parameter("vertex_radius", 1);
        this->declare_parameter("frame_path", "/home/marinarasauced/Projets/mess2/visualizer");

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
        this->get_parameter("edge_thickness", edge_thickness);
        this->get_parameter("vertex_radius", vertex_radius);
        this->get_parameter("frame_path", frame_path);

        image_path = ament_index_cpp::get_package_share_directory("mess2_visualizer_cpp") + "/" + image_file;
        image_cv2 = cv::imread(image_path, cv::IMREAD_COLOR);

        double grid_scale_x = (image_resolution - 1) / (threat_x_max - threat_x_min);
        double grid_scale_y = (image_resolution - 1) / (threat_y_max - threat_y_min);

        auto x_threat = arma::linspace(
            (threat_x_min - threat_x_min) * grid_scale_x,
            (threat_x_max - threat_x_min) * grid_scale_x,
            threat_resolution
        );
        auto y_threat = arma::linspace(
            (threat_y_min - threat_y_min) * grid_scale_y,
            (threat_y_max - threat_y_min) * grid_scale_y,
            threat_resolution
        );

        auto [x_mesh, y_mesh] = mess2_plugins::get_meshgrid(x_threat, y_threat);
        auto threat = mess2_plugins::generate_threat(x_mesh, y_mesh);

        grid_vertices = mess2_plugins::get_vertices(threat, x_mesh, y_mesh);
        grid_vertex_elements = mess2_plugins::get_threat_field_image_vertices(grid_vertices, 0.0, 0.0, 0.0);

        grid_edges = mess2_plugins::get_edges(threat);
        grid_edge_elements = mess2_plugins::get_threat_field_image_edges(grid_edges, 0.0, 0.0, 0.0);

        (void) draw_edges(grid_edge_elements, image_cv2);
        (void) draw_vertices(grid_vertex_elements, image_cv2);
        (void) reset_frame();
        (void) save_frame(frame_cv2);

        _draw_frame_server = this->create_service<ThreatDrawFrame>(
            "visualizer/draw_frame",
            std::bind(&ThreatAlgorithmFrameGenerator::draw_frame, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
private:
    void draw_edges(const ThreatElementArray& msg, cv::Mat& _cv2)
    {
        for (auto& element : msg.elements)
        {
            auto edge = grid_edges.edges[element.index];
            cv::Point p1(grid_vertices.vertices[edge.index1].position.x, grid_vertices.vertices[edge.index1].position.y);
            cv::Point p2(grid_vertices.vertices[edge.index2].position.x, grid_vertices.vertices[edge.index2].position.y);
            auto color = cv::Scalar(element.color.b * 255, element.color.g * 255, element.color.r * 255);

            cv::line(_cv2, p1, p2, color, edge_thickness);            
        }
    }

    void draw_vertices(const ThreatElementArray& msg, cv::Mat& _cv2)
    {
        for (auto& element : msg.elements)
        {
            auto vertex = grid_vertices.vertices[element.index];
            cv::Point p1(vertex.position.x, vertex.position.y);
            auto color = cv::Scalar(element.color.b * 255, element.color.g * 255, element.color.r * 255);

            cv::circle(_cv2, p1, vertex_radius, color, edge_thickness);
        } 
    }
    
    void reset_frame()
    {
        frame_cv2 = image_cv2;
    }

    void save_frame(const cv::Mat& _cv2)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << counter;
        std::string frame_file = frame_path + "/frame_" + ss.str() + ".png";
        cv::imwrite(frame_file, _cv2);
        counter = counter + 1;
    }

    void draw_frame(const std::shared_ptr<ThreatDrawFrame::Request> request, const std::shared_ptr<ThreatDrawFrame::Response> response)
    {
        try
        {
            (void) reset_frame();
            (void) draw_edges(request->edges, frame_cv2);
            (void) draw_vertices(request->vertices, frame_cv2);
            // (void) draw_occupancies(request->occupancies, frame_cv2);
            (void) save_frame(frame_cv2);
            response->success = true;
        }
        catch (const std::exception& e)
        {
            response->success = false;
            RCLCPP_INFO(this->get_logger(), "failed to save frame");
        }
        

    }

    cv::Mat image_cv2;
    cv::Mat frame_cv2;
    EdgeArray grid_edges;
    VertexArray grid_vertices;
    ThreatElementArray grid_edge_elements;
    ThreatElementArray grid_vertex_elements;
    int32_t edge_thickness;
    int32_t vertex_radius;
    std::string frame_path;
    int64_t counter = 0;
    rclcpp::Service<ThreatDrawFrame>::SharedPtr _draw_frame_server;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThreatAlgorithmFrameGenerator>());
    rclcpp::shutdown();
    return 0;
}
