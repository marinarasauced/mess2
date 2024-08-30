
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
#include "sensor_msgs/msg/image.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "mess2_msgs/msg/vertex_array.hpp"
#include "mess2_plugins/threat.hpp"

using VertexArray = mess2_msgs::msg::VertexArray;
using Image = sensor_msgs::msg::Image;

class ThreatAlgorithmImageGenerator : public rclcpp::Node
{
public:
    ThreatAlgorithmImageGenerator() : Node("algorithm_threat_gen")
    {
        int _image_resolution;
        double x_image_min;
        double x_image_max;
        double y_image_min;
        double y_image_max;
        double x_scaled_min;
        double x_scaled_max;
        double y_scaled_min;
        double y_scaled_max;

        this->declare_parameter("_image_resolution", 3001);
        this->declare_parameter("x_image_min", -15.0);
        this->declare_parameter("x_image_max", 15.0);
        this->declare_parameter("y_image_min", -15.0);
        this->declare_parameter("y_image_max", 15.0);
        this->declare_parameter("x_scaled_min", -3.0);
        this->declare_parameter("x_scaled_max", 3.0);
        this->declare_parameter("y_scaled_min", -3.0);
        this->declare_parameter("y_scaled_max", 3.0);

        this->get_parameter("_image_resolution", _image_resolution);
        this->get_parameter("x_image_min", x_image_min);
        this->get_parameter("x_image_max", x_image_max);
        this->get_parameter("y_image_min", y_image_min);
        this->get_parameter("y_image_max", y_image_max);
        this->get_parameter("x_scaled_min", x_scaled_min);
        this->get_parameter("x_scaled_max", x_scaled_max);
        this->get_parameter("y_scaled_min", y_scaled_min);
        this->get_parameter("y_scaled_max", y_scaled_max);

        RCLCPP_INFO(this->get_logger(), "generating threat image");

        arma::vec x_image = arma::linspace(x_image_min, x_image_max, _image_resolution);
        arma::vec y_image = arma::linspace(y_image_min, y_image_max, _image_resolution);
        auto [x_mesh, y_mesh] = mess2_plugins::get_meshgrid(x_image, y_image);
        image_threat = mess2_plugins::generate_threat(x_mesh, y_mesh);

        auto colormap = mess2_plugins::get_colormap(ament_index_cpp::get_package_share_directory("mess2_visualizer_cpp") + "/parula.csv");
        image_ros2 = mess2_plugins::get_threat_field_image(image_threat, colormap);

        RCLCPP_INFO(this->get_logger(), "generated threat image");

        int type;
        if (image_ros2.encoding == "mono8")
            type = CV_8UC1;
        else if (image_ros2.encoding == "bgr8")
            type = CV_8UC3;
        else if (image_ros2.encoding == "rgb8")
            type = CV_8UC3;
        else
        {
            RCLCPP_ERROR(this->get_logger(), "unsupported encoding type: %s", image_ros2.encoding.c_str());
        }
        image_cv2 = cv::Mat(image_ros2.height, image_ros2.width, type, const_cast<unsigned char *>(image_ros2.data.data()), image_ros2.step);

        cv::imwrite("threat.png", image_cv2);
        RCLCPP_INFO(this->get_logger(), "wrote threat image");
    }
private:
    arma::mat scaled_threat;
    arma::mat image_threat;
    Image image_ros2;
    cv::Mat image_cv2;
    rclcpp::Publisher<Image>::SharedPtr _image_publisher;
    rclcpp::Publisher<VertexArray>::SharedPtr _vertices_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::make_shared<ThreatAlgorithmImageGenerator>();
    rclcpp::shutdown();
    return 0;
}
