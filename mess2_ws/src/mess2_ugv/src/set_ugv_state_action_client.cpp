
// standard library headers
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

// standard ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// custom ros2 headers and plugins
#include "mess2_msgs/msg/euler_angles.hpp"
#include "mess2_msgs/msg/ugv_state.hpp"
#include "mess2_msgs/action/set_ugv_state.hpp"

// namspaces
namespace set_ugv_state_action
{
// ros2 node class
class SetUGVStateActionClient : public rclcpp::Node
{
//
public:
    // type aliases
    using Quaternion = geometry_msgs::msg::Quaternion;
    using UGVState = mess2_msgs::msg::UGVState;
    using SetUGVState = mess2_msgs::action::SetUGVState;
    using GoalHandleSetUGVState = rclcpp_action::ClientGoalHandle<SetUGVState>;

    // constructor
    explicit SetUGVStateActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("set_ugv_state_action_client", options)
    {
        //
        this->client_ptr_ = rclcpp_action::create_client<SetUGVState>(
            this,
            "set_ugv_state"
        );

        auto timer_callback_lambda = [this](){ return this->send_goal(); };
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            timer_callback_lambda
        );
    }

    //
    void send_goal()
    /* std::string name, std::string model, double k1, double k2, double speed, UGVState error_tol, UGVState vertex_init, UGVState vertex_trgt, Quaternion quat_diff */

    {
        using namespace std::placeholders;
        this->timer_->cancel();

        //
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
            rclcpp::shutdown();
        }

        // goal generation
        /* auto goal_msg = SetUGVState::Goal();
        goal_msg.name = name;
        goal_msg.model = model;
        goal_msg.k1 = k1;
        goal_msg.k2 = k2;
        goal_msg.speed = speed;
        goal_msg.error_tol = error_tol;
        goal_msg.vertex_init = vertex_init;
        goal_msg.vertex_trgt = vertex_trgt;
        goal_msg.quat_diff = quat_diff; */
        auto goal_msg = SetUGVState::Goal();
        goal_msg.name = "burger1";
        goal_msg.model = "burger";
        goal_msg.k1 = 1.0;
        goal_msg.k2 = 1.0;
        goal_msg.speed = 0.7;
        goal_msg.error_tol.state.x = 0.1;
        goal_msg.error_tol.state.y = 0.1;
        goal_msg.error_tol.state.theta = 0.1;
        goal_msg.vertex_init.state.x = 1.0;
        goal_msg.vertex_init.state.y = 0.0;
        goal_msg.vertex_init.state.theta = 0.0;
        goal_msg.vertex_trgt.state.x = 1.0;
        goal_msg.vertex_trgt.state.y = 1.0;
        goal_msg.vertex_trgt.state.theta = 0.0;
        goal_msg.quat_diff.x = 0.0;
        goal_msg.quat_diff.y = 0.0;
        goal_msg.quat_diff.z = 0.0;
        goal_msg.quat_diff.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "sending goal");

        //
        auto send_goal_options = rclcpp_action::Client<SetUGVState>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleSetUGVState::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
            }
        };

        //
        send_goal_options.feedback_callback = [this](
        GoalHandleSetUGVState::SharedPtr,
        const std::shared_ptr<const SetUGVState::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Local errors - x: " << feedback->error_local.state.x
               << ", y: " << feedback->error_local.state.y
               << ", theta: " << feedback->error_local.state.theta;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        };

        //
        send_goal_options.result_callback = [this](const GoalHandleSetUGVState::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "unknown result code");
                    return;
            }
            std::stringstream ss;
            ss << "result received: success = " << std::boolalpha << result.result->success;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        };
    }


//
private:
    // inherited from ros2 node
    rclcpp_action::Client<SetUGVState>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

}; // class SetUGVStateActionClient
} // namespace set_ugv_state_action

RCLCPP_COMPONENTS_REGISTER_NODE(set_ugv_state_action::SetUGVStateActionClient)