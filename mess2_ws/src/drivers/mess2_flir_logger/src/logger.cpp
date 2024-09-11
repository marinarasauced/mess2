
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
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "flir_camera_msgs/msg/image_meta_data.hpp"

#include <yaml-cpp/yaml.h>

class Bag2FLIR : public rclcpp::Node
{
public:
    Bag2FLIR() : Node("flir_logger")
    {
        std::string path_config;
        this->declare_parameter("path_config", "/home/mess2/mess2/mess2_ws/install/spinnaker_camera_driver/share/spinnaker_camera_driver/config/_cameras.yaml");
        this->get_parameter("path_config", path_config);

        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open("my_bag");

        YAML::Node data_config = YAML::LoadFile(path_config);
        if (data_config["cameras"]) {
            for (const auto& camera : data_config["cameras"]) {
                std::string name_camera = camera["name"].as<std::string>();

                std::string topic_camera_info = "/" + name_camera + "/camera_info";
                auto subscription_camera_info = create_subscription<sensor_msgs::msg::CameraInfo>(
                    topic_camera_info, 10, [this, topic_camera_info](std::shared_ptr<sensor_msgs::msg::CameraInfo> msg) { this->callback_camera_info(topic_camera_info, msg); });
                
                std::string topic_image_raw = "/" + name_camera + "/image_raw";
                auto subscription_image_raw = create_subscription<sensor_msgs::msg::Image>(
                    topic_image_raw, 10, [this, topic_image_raw](std::shared_ptr<sensor_msgs::msg::Image> msg) { this->callback_image_raw(topic_image_raw, msg); });

                std::string topic_meta = "/" + name_camera + "/meta";
                auto subscription_meta = create_subscription<flir_camera_msgs::msg::ImageMetaData>(
                    topic_meta, 10, [this, topic_meta](std::shared_ptr<flir_camera_msgs::msg::ImageMetaData> msg) { this->callback_meta(topic_meta, msg); });

                subscriptions_camera_info_[topic_camera_info] = subscription_camera_info;
                subscriptions_image_raw_[topic_image_raw] = subscription_image_raw;
                subscriptions_meta_[topic_meta] = subscription_meta;
            }
        }
    }
private:
    void callback_camera_info(const std::string& name_topic, std::shared_ptr<sensor_msgs::msg::CameraInfo> msg) const
    {
        rclcpp::SerializedMessage msg_;
        serializer_camera_info_.serialize_message(&msg, &msg_);
        auto msg_ptr = std::make_shared<rclcpp::SerializedMessage>(msg_);

        rclcpp::Time time_stamp = this->now();
        writer_->write(msg_ptr, name_topic, "sensor_msgs/msg/CameraInfo", time_stamp);
    }

    void callback_image_raw(const std::string& name_topic, std::shared_ptr<sensor_msgs::msg::Image> msg) const
    {
        rclcpp::SerializedMessage msg_;
        serializer_image_raw_.serialize_message(&msg, &msg_);
        auto msg_ptr = std::make_shared<rclcpp::SerializedMessage>(msg_);

        rclcpp::Time time_stamp = this->now();
        writer_->write(msg_ptr, name_topic, "sensor_msgs/msg/Image", time_stamp);
    }

    void callback_meta(const std::string& name_topic, std::shared_ptr<flir_camera_msgs::msg::ImageMetaData> msg) const
    {
        rclcpp::SerializedMessage msg_;
        serializer_meta_.serialize_message(&msg, &msg_);
        auto msg_ptr = std::make_shared<rclcpp::SerializedMessage>(msg_);

        rclcpp::Time time_stamp = this->now();
        writer_->write(msg_ptr, name_topic, "flir_camera_msgs/msg/ImageMetaData", time_stamp);
    }

    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serializer_camera_info_;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer_image_raw_;
    rclcpp::Serialization<flir_camera_msgs::msg::ImageMetaData> serializer_meta_;

    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subscriptions_camera_info_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_image_raw_;
    std::unordered_map<std::string, rclcpp::Subscription<flir_camera_msgs::msg::ImageMetaData>::SharedPtr> subscriptions_meta_;

    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bag2FLIR>());;
    rclcpp::shutdown();
    return 0;
}
