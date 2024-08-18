
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

#include "mess2_plugins/utils.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using CommandBool = mavros_msgs::srv::CommandBool;
using SetMode = mavros_msgs::srv::SetMode;

using namespace mess2_plugins;
namespace mess2_nodes
{
class UAVOffboardNode : public rclcpp::Node
{
public:
    explicit UAVOffboardNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("uav_offboard", options)
    {
        using namespace std::placeholders;

        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);
        _vicon_topic = get_vicon_topic(agent_name_);
        _vision_pose_topic = "vision_pose/pose";

        _vision_pose_publisher = this->create_publisher<PoseStamped>(
            _vision_pose_topic,
            10
        );

        _vicon_subscription = this->create_subscription<TransformStamped>(
            _vicon_topic,
            10,
            std::bind(&UAVOffboardNode::_vicon_callback, this, _1)
        );

    }
private:
    std::string agent_name_;
    std::string _vicon_topic;
    std::string _vision_pose_topic;

    // TransformStamped global_;

    rclcpp::Publisher<PoseStamped>::SharedPtr _vision_pose_publisher;
    rclcpp::Subscription<TransformStamped>::SharedPtr _vicon_subscription;

    void _vicon_callback(const TransformStamped::SharedPtr msg)
    {
        // global_.header = msg->header;
        // global_.transform = msg->transform;

        PoseStamped msg_;
        msg_.header = msg->header;
        msg_.pose.position.x = msg->transform.translation.x;
        msg_.pose.position.y = msg->transform.translation.y;
        msg_.pose.position.z = -msg->transform.translation.z;  // invert z bc mavros uses FRD
        msg_.pose.orientation = msg->transform.rotation;
        _vision_pose_publisher->publish(msg_);
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_nodes::UAVOffboardNode)
