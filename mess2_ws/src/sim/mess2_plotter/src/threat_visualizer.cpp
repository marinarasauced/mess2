
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
        int resolution = 301;
        arma::vec x_ = arma::linspace(-15.0, 15.0, resolution);
        arma::vec y_ = arma::linspace(-15.0, 15.0, resolution);
        arma::vec x_scaled = arma::linspace(-3.0, 3.0, resolution);
        arma::vec y_scaled = arma::linspace(-3.0, 3.0, resolution);
        auto [x_mesh, y_mesh] = mess2_plugins::get_meshgrid(x_, y_);
        threat = mess2_plugins::generate_threat(x_mesh, y_mesh);

        auto colormap = get_colormap(ament_index_cpp::get_package_share_directory("mess2_plotter") + "/parula.csv";)
        auto image = get_threat_field_image(threat, colormap)

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
