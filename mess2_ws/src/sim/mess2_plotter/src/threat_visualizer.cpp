
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
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mess2_plugins/threat.hpp"

using Image = sensor_msgs::msg::Image;

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

        auto colormap = mess2_plugins::get_colormap(ament_index_cpp::get_package_share_directory("mess2_plotter") + "/parula.csv");
        image = mess2_plugins::get_threat_field_image(threat, colormap);

        _image_publisher = this->create_publisher<Image>(
            "mess2/visualizer/threat/raw",
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
        _image_publisher->publish(image);
    }

    arma::mat threat;
    Image image;
    Image image_;
    rclcpp::Publisher<Image>::SharedPtr _image_publisher;
    rclcpp::TimerBase::SharedPtr _image_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlgorithmThreatVisualizer>());
    rclcpp::shutdown();
    return 0;
}
