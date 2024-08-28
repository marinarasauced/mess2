
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "mess2_msgs/msg/euler_angles.hpp"
#include "mess2_msgs/msg/ugv_state.hpp"
#include "mess2_msgs/action/ugv_face_time_invariant_heading.hpp"
#include "mess2_plugins/rotation.hpp"
#include "mess2_plugins/ugv.hpp"
#include "mess2_plugins/utils.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;
using Twist = geometry_msgs::msg::Twist;
using Quaternion = geometry_msgs::msg::Quaternion;
using EulerAngles = mess2_msgs::msg::EulerAngles;
using State = mess2_msgs::msg::UGVState;
using Action = mess2_msgs::action::UGVFaceTimeInvariantHeading;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

using namespace mess2_plugins;
namespace mess2_nodes
{
class UGVTimeInvariantHeadingFacingServer : public rclcpp::Node
{
public:
    explicit UGVTimeInvariantHeadingFacingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("face_time_invariant_heading_server", options)
    {
        using namespace std::placeholders;

        _cmd_vel_topic = get_cmd_vel_topic();
        _cmd_vel_publisher = this->create_publisher<Twist>(
            _cmd_vel_topic,
            10
        );

        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);
        _vicon_topic = get_vicon_topic(agent_name_);
        _vicon_subscription = this->create_subscription<TransformStamped>(
            _vicon_topic,
            10,
            std::bind(&UGVTimeInvariantHeadingFacingServer::_vicon_callback, this, _1)
        );

        this->declare_parameter<std::string>("model", "burger");
        this->declare_parameter<double>("k_lin", 1.0);
        this->declare_parameter<double>("k_ang", 1.0);
        this->declare_parameter<double>("speed", 0.7);
        this->declare_parameter<std::vector<double>>("tolerance", {0.1, 0.1, 0.1});

        this->get_parameter("model", model_);
        this->get_parameter("k_lin", k_lin_);
        this->get_parameter("k_ang", k_ang_);
        this->get_parameter("speed", speed_);
        max_u_lin_ = get_max_u_lin(model_, speed_);
        max_u_ang_ = get_max_u_ang(model_, speed_);
        std::vector<double> tolerance;
        this->get_parameter("tolerance", tolerance);
        tolerance_.state.x = tolerance[0];
        tolerance_.state.y = tolerance[1];
        tolerance_.state.theta = tolerance[2];

        auto handle_goal = [this](
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Action::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "received goal request");
            (void)uuid;

            double dtheta = std::abs(wrap_to_pi(global_ - goal->trgt));
            if (dtheta < tolerance_.state.theta)
            {
                RCLCPP_INFO(this->get_logger(), "trgt too similar to global");
                return rclcpp_action::GoalResponse::REJECT;
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel = [this](
            const std::shared_ptr<GoalHandle> goal_handel)
        {
            RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
            (void)goal_handel;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted = [this](
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle](){return this->_face_time_invariant_heading_execute(goal_handle);};
            std::thread{execute_in_thread}.detach();
        };

        this->_face_time_invariant_heading_server = rclcpp_action::create_server<Action>(
            this,
            "face_time_invariant_heading",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }
private:
    void _cmd_vel_callback(const double u_lin, const double u_ang)
    {
        Twist msg;
        msg.linear.x = u_lin;
        msg.angular.z = u_ang;
        if (std::abs(msg.linear.x) > max_u_lin_)
        {
            msg.linear.x = std::copysign(max_u_lin_, u_lin);
        }
        if (std::abs(msg.angular.z) > max_u_ang_)
        {
            msg.angular.z = std::copysign(max_u_ang_, u_ang);
        }
        _cmd_vel_publisher->publish(msg);
    }

    void _vicon_callback(const TransformStamped::SharedPtr msg)
    {
        EulerAngles euler = convert_quat_to_eul(msg->transform.rotation);
        global_= euler.yaw;

        auto error = get_error_from_heading(global_, trgt_);
        error_ = error;
    }

    void _face_time_invariant_heading_execute(std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "executing face heading goal");
        rclcpp::Rate rate(10);

        auto goal = goal_handle->get_goal();
        trgt_ = goal->trgt;

        auto feedback = std::make_shared<Action::Feedback>();
        auto result = std::make_shared<Action::Result>();

        error_ = std::numeric_limits<float>::infinity();
        while (error_ == std::numeric_limits<float>::infinity())
        {
            rate.sleep();
        }

        while (rclcpp::ok() && std::abs(error_) > tolerance_.state.theta) 
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "face heading goal cancelled");
                return;
            }
            double u_lin = 0.0;
            double u_ang = -k_ang_ * error_;
            (void) _cmd_vel_callback(u_lin, u_ang);
            feedback->error = error_;
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }
        (void) _cmd_vel_callback(0.0, 0.0);

        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "face heading goal succeeded");
        }
    }

    double error_;
    double global_;
    double trgt_;

    std::string _cmd_vel_topic;
    rclcpp::Publisher<Twist>::SharedPtr _cmd_vel_publisher;

    std::string agent_name_;
    std::string _vicon_topic;
    rclcpp::Subscription<TransformStamped>::SharedPtr _vicon_subscription;

    std::string model_;
    double k_lin_;
    double k_ang_;
    double speed_;
    double max_u_lin_;
    double max_u_ang_;
    State tolerance_;

    rclcpp_action::Server<Action>::SharedPtr _face_time_invariant_heading_server;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_nodes::UGVTimeInvariantHeadingFacingServer)
