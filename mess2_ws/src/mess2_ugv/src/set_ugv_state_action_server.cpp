
// library standard headers
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>

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
#include "mess2_msgs/action/set_ugv_state.hpp"
#include "mess2_plugins/quaternions.hpp"

using namespace mess2_plugins;
namespace set_ugv_state_action
{
// ros2 node class
class SetUGVStateActionServer : public rclcpp::Node
{
public:

    // type aliases
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using Twist = geometry_msgs::msg::Twist;
    using Quaternion = geometry_msgs::msg::Quaternion;
    using EulerAngles = mess2_msgs::msg::EulerAngles;
    using UGVState = mess2_msgs::msg::UGVState;
    using SetUGVState = mess2_msgs::action::SetUGVState;
    using GoalHandleSetUGVState = rclcpp_action::ServerGoalHandle<SetUGVState>;

    // constructor
    explicit SetUGVStateActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("set_ugv_state_action_server", options)
    {
        // namespaces
        using namespace std::placeholders;

        // handle goal requests
        auto handle_goal = [this](
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const SetUGVState::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "received goal request");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        // handle requests to cancel goal
        auto handle_cancel = [this](
            const std::shared_ptr<GoalHandleSetUGVState> goal_handel)
        {
            RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
            (void)goal_handel;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        // handle accepted goals
        auto handle_accepted = [this](
            const std::shared_ptr<GoalHandleSetUGVState> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle](){return this->execute_goal(goal_handle);};
            std::thread{execute_in_thread}.detach();
        };

        // create action server
        this->action_server_ = rclcpp_action::create_server<SetUGVState>(
            this,
            "set_ugv_state",
            handle_goal,
            handle_cancel,
            handle_accepted);

        // publishers and subscribers
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_vicon_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("vicon", 10, std::bind(&SetUGVStateActionServer::callback_vicon, this, std::placeholders::_1));

    }

private:

    //
    rclcpp_action::Server<SetUGVState>::SharedPtr action_server_;
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
    UGVState error_tol_;

    // initial and target vertices
    UGVState vertex_init_;
    UGVState vertex_trgt_;

    // difference quaternion
    Quaternion quat_diff_;

    // global state and local error
    UGVState state_global_;
    UGVState error_local_;

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
    void execute_goal(std::shared_ptr<GoalHandleSetUGVState> goal_handle) {

        //
        RCLCPP_INFO(this->get_logger(), "executing goal");
        rclcpp::Rate loop_rate(10);

        // load goal, define feedback and result
        (void)load_goal(goal_handle);
        auto result = std::make_shared<SetUGVState::Result>();
        auto feedback = std::make_shared<SetUGVState::Feedback>();

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
    void load_goal(std::shared_ptr<GoalHandleSetUGVState> goal_handle) {

        // get goal
        const auto goal = goal_handle->get_goal();

        // vehicle identification
        name_ = goal->name;
        model_ = goal->model;

        // control parameters
        k1_ = goal->k1;
        k2_ = goal->k2;
        double speed = goal->speed;
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
        error_tol_ = goal->error_tol;

        // initial and target vertices
        vertex_init_ = goal->vertex_init;
        vertex_trgt_ = goal->vertex_trgt;

        // difference quaternion
        quat_diff_ = goal->quat_diff;
    }

}; // class SetUGVStateActionServer
} // namespace set_ugv_state_action

RCLCPP_COMPONENTS_REGISTER_NODE(set_ugv_state_action::SetUGVStateActionServer)
