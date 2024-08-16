
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
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mess2_msgs/action/uav_calibrate.hpp"

using Action = mess2_msgs::action::UAVCalibrate;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

namespace mess2_actions
{
class UAVCalibrationClient : public rclcpp::Node
{
public:
    explicit UAVCalibrationClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("uav_calibration_client", options)
    {
        this->declare_parameter("num_measurements", 1000);

        this->_calibration_client = rclcpp_action::create_client<Action>(
            this,
            "calibrate_uav"
        );

        this->_calibration_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UAVCalibrationClient::send_goal, this)
        );

    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->_calibration_timer->cancel();

        if (!this->_calibration_client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = Action::Goal();
        this->get_parameter("num_measurements", goal_msg.num_measurements);

        RCLCPP_INFO(this->get_logger(), "sending goal");

        auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
        std::bind(&UAVCalibrationClient::_goal_response_callback, this, _1);
        
        send_goal_options.feedback_callback =
        std::bind(&UAVCalibrationClient::_feedback_callback, this, _1, _2);
        
        send_goal_options.result_callback =
        std::bind(&UAVCalibrationClient::result_callback, this, _1);
        
        this->_calibration_client->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Action>::SharedPtr _calibration_client;
    rclcpp::TimerBase::SharedPtr _calibration_timer;

    void _goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
        }
    }

    void _feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Action::Feedback> feedback)
    {
        auto progress = feedback->progress;
        RCLCPP_INFO(this->get_logger(), "progress: %.2f%%", progress);
    }

    void result_callback(const GoalHandle::WrappedResult & result)
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
        ss << "calibration quaternion received:\n";
        ss << "x: " << result.result->quat_diff.x << "\n";
        ss << "y: " << result.result->quat_diff.y << "\n";
        ss << "z: " << result.result->quat_diff.z << "\n";
        ss << "w: " << result.result->quat_diff.w << "\n";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};  
}

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_actions::UAVCalibrationClient)
