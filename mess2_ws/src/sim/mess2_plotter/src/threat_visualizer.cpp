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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <visualization_msgs/msg/image_marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.hpp>
#include <opencv2/opencv.hpp>

#include "mess2_plugins/threat.hpp"

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class AlgorithmThreatVisualizer : public rclcpp::Node
{
public:
    AlgorithmThreatVisualizer() : Node("threat_visualizer")
    {
        RCLCPP_INFO(this->get_logger(), "generating threat field");
        int resolution = 301;
        arma::vec x_ = arma::linspace(-15.0, 15.0, resolution);
        arma::vec y_ = arma::linspace(-15.0, 15.0, resolution);
        arma::vec x_scaled = arma::linspace(-3.0, 3.0, resolution);
        arma::vec y_scaled = arma::linspace(-3.0, 3.0, resolution);
        auto [x_mesh, y_mesh] = mess2_plugins::get_meshgrid(x_, y_);
        threat = mess2_plugins::generate_threat(x_mesh, y_mesh);
        RCLCPP_INFO(this->get_logger(), "generated threat field");

        RCLCPP_INFO(this->get_logger(), "loading colormap");
        std::string colormap_dir = ament_index_cpp::get_package_share_directory("mess2_plotter");
        std::string colormap_file = colormap_dir + "/colormap.csv";
        std::ifstream file(colormap_file);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "failed to open colormap file");
            return;
        }

        std::vector<std::array<double, 3>> colormap;
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            std::string item;
            std::array<double, 3> color;

            std::getline(ss, item, ',');
            color[0] = std::stof(item);
            std::getline(ss, item, ',');
            color[1] = std::stof(item);
            std::getline(ss, item, ',');
            color[2] = std::stof(item);

            if (color[0] < 0.0f || color[0] > 1.0f ||
            color[1] < 0.0f || color[1] > 1.0f ||
            color[2] < 0.0f || color[2] > 1.0f)
            {
                RCLCPP_WARN(this->get_logger(), "colormap value out of range");
            }
            colormap.push_back(color);
        }
        // for (const auto& color : colormap)
        // {
        //     RCLCPP_INFO(this->get_logger(), "colormap color: r=%f, g=%f, b=%f", color[0], color[1], color[2]);
        // }
        // RCLCPP_INFO(this->get_logger(), "loaded colormap");

        RCLCPP_INFO(this->get_logger(), "creating image");
        image.header.stamp = this->get_clock()->now();
        image.header.frame_id = "map";
        int32_t width = threat.n_cols;
        int32_t height = threat.n_rows;
        image.width = width;
        image.height = height;
        image.encoding = "rgb8";
        image.step = image.width * 3;
        image.data.resize(image.width * image.height * 3);
        double min_threat = threat.min();
        double max_threat = threat.max();
        for (int32_t iter = 0; iter < height; ++iter)
        {
            for (int32_t jter = 0; jter < width; ++jter)
            {
                double value = threat(iter, jter);
                double norm_value = (value - min_threat) / (max_threat - min_threat);
                norm_value = std::max(0.0, std::min(1.0, norm_value));

                uint8_t r, g, b;
                int index1 = static_cast<int>(norm_value * (colormap.size() - 1));
                index1 = std::max(0, std::min(index1, static_cast<int>(colormap.size()) - 1));
                const auto &color = colormap[index1];
                r = color[0] * 255;
                g = color[1] * 255;
                b = color[2] * 255;

                size_t index2 = (iter * image.width + jter) * 3;
                image.data[index2] = r;
                image.data[index2 + 1] = g;
                image.data[index2 + 2] = b;
            }
        }

        cv_bridge::CvImagePtr cv_ptr;
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
        } 
        catch (cv_bridge::Exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "cv bridge exception: %s", e.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "created image");

        RCLCPP_INFO(this->get_logger(), "publishing image to rviz");
        _image_publisher = this->create_publisher<PointCloud2>(
            "/mess2/visualizer/threat",
            10
        );

        _image_timer = this->create_timer(
            std::chrono::seconds(1),
            [this]() {this->_image_callback(); }
        );
    }
private:
    void _image_callback()
    {
        _image_publisher->publish(pc2);
    }

    arma::mat threat;
    Image image;
    PointCloud2 pc2;
    rclcpp::Publisher<PointCloud2>::SharedPtr _image_publisher;
    rclcpp::TimerBase::SharedPtr _image_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlgorithmThreatVisualizer>());
    rclcpp::shutdown();
    return 0;
}
