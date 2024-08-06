
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

// library third-party headers
#include </usr/include/armadillo>

// ros2 standard headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// ros2 custom headers and plugins
#include "mess2_msgs/msg/euler_angles.hpp"
#include "mess2_msgs/msg/ugv_state.hpp"
#include "mess2_msgs/action/ugv_follow_line.hpp"
#include "mess2_plugins/angles.hpp"
#include "mess2_plugins/quaternions.hpp"

using namespace mess2_plugins;
namespace ugv_actions
{
// ros2 node class
class UGVFollowLineActionServer : public rclcpp::Node
{
public:

    // type aliases
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using Twist = geometry_msgs::msg::Twist;
    using Quaternion = geometry_msgs::msg::Quaternion;
    using EulerAngles = mess2_msgs::msg::EulerAngles;
    using State = mess2_msgs::msg::UGVState;
    using FollowLine = mess2_msgs::action::UGVFollowLine;
    using GoalHandleUGVFollowLine = rclcpp_action::ServerGoalHandle<FollowLine>;

    // constructor
    explicit UGVFollowLineActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ugv_line_following_server", options)
    {
        // namespaces
        using namespace std::placeholders;

        // declare parameters
        this->declare_parameter<std::string>("model", "burger");
        this->declare_parameter<std::string>("name", "ugv");
        this->declare_parameter<double>("k1", 1.0);
        this->declare_parameter<double>("k2", 1.0);
        this->declare_parameter<double>("speed", 0.7);
        this->declare_parameter<double>("error_tol_x", 0.1);
        this->declare_parameter<double>("error_tol_y", 0.1);
        this->declare_parameter<double>("error_tol_theta", 0.1);
        this->declare_parameter<double>("quat_diff_x", 0.0);
        this->declare_parameter<double>("quat_diff_y", 0.0);
        this->declare_parameter<double>("quat_diff_z", 0.0);
        this->declare_parameter<double>("quat_diff_w", 1.0);

        // handle goal requests
        auto handle_goal = [this](
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const FollowLine::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "received goal request");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        // handle requests to cancel goal
        auto handle_cancel = [this](
            const std::shared_ptr<GoalHandleUGVFollowLine> goal_handel)
        {
            RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
            (void)goal_handel;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        // handle accepted goals
        auto handle_accepted = [this](
            const std::shared_ptr<GoalHandleUGVFollowLine> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle](){return this->execute_goal(goal_handle);};
            std::thread{execute_in_thread}.detach();
        };

        // create action server
        this->action_server_ = rclcpp_action::create_server<FollowLine>(
            this,
            "ugv_follow_line",
            handle_goal,
            handle_cancel,
            handle_accepted);

        // publishers and subscribers
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_vicon_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("vicon", 10, std::bind(&UGVFollowLineActionServer::callback_vicon, this, std::placeholders::_1));
    }

private:

    //
    rclcpp_action::Server<FollowLine>::SharedPtr action_server_;
    rclcpp::Publisher<Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<TransformStamped>::SharedPtr sub_vicon_;

    // vehicle identification
    std::string name_;
    std::string model_;

    // control parameters
    double k1_;
    double k2_;
    double max_lin_vel_;
    double max_ang_vel_;

    // error tolerance
    State error_tol_;

    // initial and target vertices
    State vertex_init_;
    State vertex_trgt_;

    // difference quaternion
    Quaternion quat_diff_;

    // global state and local error
    State state_global_;
    State error_local_;

    // localization status
    bool status_ = false;

    // callback function for command velocity publisher
    void callback_cmd_vel(const double u_lin, const double u_ang) {

        // message generation
        Twist msg;
        msg.linear.x = u_lin;
        msg.angular.z = u_ang;

        // message value constraint
        if (std::abs(msg.linear.x) > max_lin_vel_)
        {
            msg.linear.x = std::copysign(max_lin_vel_, u_lin);
        }
        if (std::abs(msg.angular.z) > max_ang_vel_)
        {
            msg.angular.z = std::copysign(max_ang_vel_, u_ang);
        }

        // message publication
        pub_cmd_vel_->publish(msg);
    }

    // callback function for vicon localization subscription
    void callback_vicon(const TransformStamped::SharedPtr msg) {

        // header
        state_global_.header.stamp = msg->header.stamp;

        // global theta
        Quaternion quat_meas = msg->transform.rotation;
        Quaternion quat_true = multiply_two_quats(quat_diff_, quat_meas);
        EulerAngles eul_true = convert_quat_to_eul(quat_true);
        state_global_.state.theta = eul_true.yaw;

        // global x and y
        state_global_.state.x = msg->transform.translation.x;
        state_global_.state.y = msg->transform.translation.y;

        // local error
        arma::vec A = {vertex_trgt_.state.x - vertex_init_.state.x, vertex_trgt_.state.y - vertex_init_.state.y};
        arma::vec B = {vertex_trgt_.state.x - state_global_.state.x, vertex_trgt_.state.y - state_global_.state.y};
        arma::vec C = {state_global_.state.x - vertex_init_.state.x, state_global_.state.y - vertex_init_.state.y};
        double a = arma::norm(A);
        double b = arma::norm(B);
        double alpha = std::acos(arma::dot(A, B) / (a * b));
        double beta = std::atan2(C(1), C(0));
        double theta = std::atan2(A(1), A(0));
        error_local_.state.x = b * std::cos(alpha);
        error_local_.state.y = b * std::sin(alpha) * std::copysign(1.0, beta - theta);
        error_local_.state.theta = wrap_to_pi(state_global_.state.theta - theta);

        //
        status_ = true;
    }

    // execute action
    void execute_goal(std::shared_ptr<GoalHandleUGVFollowLine> goal_handle) {

        //
        RCLCPP_INFO(this->get_logger(), "executing goal");
        rclcpp::Rate loop_rate(10);

        // load goal, define feedback and result
        (void)load_goal(goal_handle);
        (void)load_parameters();
        auto result = std::make_shared<FollowLine::Result>();
        auto feedback = std::make_shared<FollowLine::Feedback>();

        // set error to inf 
        error_local_.state.x = std::numeric_limits<float>::infinity();
        error_local_.state.y = std::numeric_limits<float>::infinity();
        error_local_.state.theta = std::numeric_limits<float>::infinity();

        // wait for localization
        while (rclcpp::ok() && status_ == false) {
            RCLCPP_INFO(this->get_logger(), "waiting for localization");
            loop_rate.sleep();
        }

        // rotate ugv towards target vertex
        while (rclcpp::ok() && std::abs(error_local_.state.theta) > error_tol_.state.theta) {

            // if goal is cancelled
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal cancelled");
                return;
            }

            // control ugv
            double u_ang = -k2_ * error_local_.state.theta;
            (void)callback_cmd_vel(0.0, u_ang);
            RCLCPP_INFO(this->get_logger(), "rotation");

            // sleep
            loop_rate.sleep();
        }
        (void)callback_cmd_vel(0.0, 0.0);

        // translate ugv to target vertex
        while (rclcpp::ok() && std::abs(error_local_.state.x) > error_tol_.state.x) {

            // if goal is cancelled
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal cancelled");
                return;
            }

            // control ugv
            double u_ang = -k1_ * error_local_.state.y -k2_ * error_local_.state.theta;
            (void)callback_cmd_vel(max_lin_vel_, u_ang);
            RCLCPP_INFO(this->get_logger(), "translation");

            // sleep
            loop_rate.sleep();
        }
        (void)callback_cmd_vel(0.0, 0.0);

        //
        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    }

    // load goal
    void load_goal(std::shared_ptr<GoalHandleUGVFollowLine> goal_handle)
    {

        // get goal
        const auto goal = goal_handle->get_goal();

        // initial vertex
        vertex_init_.state.x = goal->init_x;
        vertex_init_.state.y = goal->init_y;

        // target vertex
        vertex_trgt_.state.x = goal->trgt_x;
        vertex_trgt_.state.y = goal->trgt_y;
    }

    // load parameters
    void load_parameters()
    {
        // vehicle identification
        this->get_parameter("model", model_);
        this->get_parameter("name", name_);

        // vehicle control
        this->get_parameter("k1", k1_);
        this->get_parameter("k2", k2_);
        double speed;
        this->get_parameter("speed", speed);
        if (model_ == "burger") {
            max_lin_vel_ = speed * 0.22;
            max_ang_vel_ = speed * 2.84;
        } else if (model_ == "wafflepi") {
            max_lin_vel_ = speed * 0.26;
            max_ang_vel_ = speed * 1.82;
        } else {
            max_lin_vel_ = speed * 0.22;
            max_ang_vel_ = speed * 1.82;  
        }

        // error tolerance
        this->get_parameter("error_tol_x", error_tol_.state.x);
        this->get_parameter("error_tol_y", error_tol_.state.y);
        this->get_parameter("error_tol_theta", error_tol_.state.theta);

        // calibration quaternion
        this->get_parameter("quat_diff_x", quat_diff_.x);
        this->get_parameter("quat_diff_y", quat_diff_.y);
        this->get_parameter("quat_diff_z", quat_diff_.z);
        this->get_parameter("quat_diff_w", quat_diff_.w);
    }

}; // class UGVFollowLineActionServer
} // namespace ugv_actions

RCLCPP_COMPONENTS_REGISTER_NODE(ugv_actions::UGVFollowLineActionServer)