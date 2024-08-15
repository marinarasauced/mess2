
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

// ros2 standard library headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// ros2 custom headers and plugins
#include "mess2_plugins/orientation.hpp"

//
using namespace mess2_plugins;

//ros2 node class
class HawkDiagnostics : public rclcpp::Node
{
public:

    // type aliases
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using Quaternion = geometry_msgs::msg::Quaternion;

    // constructor
    HawkDiagnostics() : Node("hawk_diagnostics")
    {
        // namespaces
        using namespace std::placeholders;

        // declare and retrieve parameters
        this->declare_parameter("name", "uav");
        this->declare_parameter<double>("quat_diff_x", 0.0);
        this->declare_parameter<double>("quat_diff_y", 0.0);
        this->declare_parameter<double>("quat_diff_z", 0.0);
        this->declare_parameter<double>("quat_diff_w", 1.0);

        this->get_parameter("name", name_);
        this->get_parameter("quat_diff_x", quat_diff_.x);
        this->get_parameter("quat_diff_y", quat_diff_.y);
        this->get_parameter("quat_diff_z", quat_diff_.z);
        this->get_parameter("quat_diff_w", quat_diff_.w);

        //
        vicon_topic_raw_ = "/vicon/" + name_ + "/" + name_;
        vicon_topic_calibrated_ = vicon_topic_raw_ + "/calibrated";
        vicon_topic_vision_pose_ = "vision_pose/pose";

        //
        subsciption_vicon_ = this->create_subscription<TransformStamped>(vicon_topic_raw_, 10, std::bind(&HawkDiagnostics::callback_vicon_, this, std::placeholders::_1));
        publisher_vicon_ = this->create_publisher<TransformStamped>(vicon_topic_calibrated_, 10);
    };

private:

    //
    std::string vicon_topic_raw_;
    std::string vicon_topic_calibrated_;
    std::string vicon_topic_vision_pose_;
    rclcpp::Subscription<TransformStamped>::SharedPtr subsciption_vicon_;
    rclcpp::Publisher<TransformStamped>::SharedPtr publisher_vicon_;
    rclcpp::Publisher<PoseStamped>::SharedPtr publisher_vision_pose_;

    // vehicle identification
    std::string name_;

    // difference quaternion
    Quaternion quat_diff_;

    // callback function for vicon localization subscription
    void callback_vicon_(const TransformStamped::SharedPtr msg)
    {
        // 
        Quaternion quat_meas = msg->transform.rotation;
        Quaternion quat_true = multiply_two_quats(quat_diff_, quat_meas);

        // 
        TransformStamped msg_calibrated;
        msg_calibrated.header = msg->header;
        msg_calibrated.transform.translation = msg->transform.translation;
        msg_calibrated.transform.rotation = quat_true;
        publisher_vicon_->publish(msg_calibrated);

        //
        PoseStamped msg_vision_pose;
        msg_vision_pose.header = msg->header;
        msg_vision_pose.pose.position.x = msg->transform.translation.x;
        msg_vision_pose.pose.position.y = msg->transform.translation.y;
        msg_vision_pose.pose.position.z = msg->transform.translation.z;
        msg_vision_pose.pose.orientation = quat_true;
        publisher_vision_pose_->publish(msg_vision_pose);
    }
};

//
int main(int argc, char * argv[])
{
    //
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HawkDiagnostics>());
    rclcpp::shutdown();
    return 0;
}