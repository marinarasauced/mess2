
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class LogTopicsToJPG : public rclcpp::Node
{
public:
    LogTopicsToJPG() : Node("log_to_jpg")
    {
        std::string path_config;
        this->declare_parameter("path_config", "/home/mess2/mess2/mess2_ws/install/spinnaker_camera_driver/share/spinnaker_camera_driver/config/_cameras.yaml");
        this->get_parameter("path_config", path_config);

        
        this->declare_parameter("dir_sub", "flir");
        this->declare_parameter("dir_logs", "/home/mess2/mess2/logs/2024_09_12/testing");
        this->get_parameter("dir_sub", dir_sub_);
        this->get_parameter("dir_logs", dir_logs_);

        std::filesystem::path dir_path = std::filesystem::path(dir_logs_) / std::filesystem::path(dir_sub_);
        if (!std::filesystem::exists(dir_path)) {
            if (!std::filesystem::create_directories(dir_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create directories: %s", dir_path.string().c_str());
                return;
            }
        }

        YAML::Node data_config = YAML::LoadFile(path_config);
        if (data_config["cameras"]) {
            for (const auto& camera : data_config["cameras"]) {
                std::string name_camera = camera["name"].as<std::string>();
                std::string topic_image_raw = "/" + name_camera + "/image_raw";

                auto subscription_image_raw = create_subscription<sensor_msgs::msg::Image>(topic_image_raw, 10, [this, name_camera](std::shared_ptr<sensor_msgs::msg::Image> msg) { this->callback_image_raw(name_camera, msg); });

                subscriptions_image_raw_[topic_image_raw] = subscription_image_raw;
            }
        }
    }

private:
    void callback_image_raw(const std::string& camera_name, std::shared_ptr<sensor_msgs::msg::Image> msg) const
    {
        try {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

            rclcpp::Time timestamp = msg->header.stamp;
            std::ostringstream filename;
            filename << dir_logs_ << "/" << dir_sub_ << "/" << camera_name << "/";
            filename << camera_name << "_";
            filename << std::setw(10) << std::setfill('0') << timestamp.nanoseconds();
            filename << ".jpg";

            cv::imwrite(filename.str(), cv_ptr->image);
            
        } catch (const cv_bridge::Exception& e) {
            // RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_image_raw_;
    std::string dir_sub_;
    std::string dir_logs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogTopicsToJPG>());;
    rclcpp::shutdown();
    return 0;
}
