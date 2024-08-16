
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

#include </usr/include/armadillo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "mess2_msgs/action/uav_calibrate.hpp"
#include "mess2_plugins/calibration.hpp"
#include "mess2_plugins/utils.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;
using Action = mess2_msgs::action::UAVCalibrate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

using namespace mess2_plugins;
namespace mess2_actions
{
class UAVCalibrationServer : public rclcpp::Node
{
public:
    explicit UAVCalibrationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("uav_calibration_server", options)
    {
        using namespace std::placeholders;

        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);
        _vicon_topic = get_vicon_topic(agent_name_);

        this->create_subscription<TransformStamped>(
            _vicon_topic,
            10,
            std::bind(&UAVCalibrationServer::_vicon_callback, this, _1)
        );

        this->_calibration_server = rclcpp_action::create_server<Action>(
            this,
            "calibrate_uav",
            std::bind(&UAVCalibrationServer::_handle_goal, this, _1, _2),
            std::bind(&UAVCalibrationServer::_handle_cancel, this, _1),
            std::bind(&UAVCalibrationServer::_handle_accepted, this, _1)
        );
    }

private:
    std::string agent_name_;
    TransformStamped global_;

    std::string _vicon_topic;
    rclcpp::Subscription<TransformStamped>::SharedPtr _vicon_subscription;
    rclcpp_action::Server<Action>::SharedPtr _calibration_server;

    rclcpp_action::GoalResponse _handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "received goal request");
        (void)uuid;
        if (goal->num_measurements > 0)
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            return rclcpp_action::GoalResponse::REJECT; 
        }
        
    }

    rclcpp_action::CancelResponse _handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void _handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&UAVCalibrationServer::_calibration_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void _calibration_execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "executing goal");
        rclcpp::Rate rate(20);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Action::Feedback>();
        auto result = std::make_shared<Action::Result>();

        TransformStamped meas1;
        TransformStamped meas2;
        int64_t counter1 = 0;
        int64_t counter2 = 0;
        rclcpp::Time toc = global_.header.stamp;

        RCLCPP_INFO(this->get_logger(), "move the hawk to pose one then press enter to collect measurements");
        std::cin.get();
        RCLCPP_INFO(this->get_logger(), "collecting measurements at pose one");

        while (rclcpp::ok() && counter1 < goal->num_measurements)
        {
            if (goal_handle->is_canceling()) {
                result->success = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal canceled");
                return;
            }
            rclcpp::Time tic = global_.header.stamp;
            if (tic > toc)
            {
                meas1.header.stamp = tic;
                meas1.transform = mess2_plugins::update_measurement(meas1.transform, global_.transform, counter1);
                counter1 = counter1 + 1;
                toc = tic;
                
                feedback->progress = 0.0 * (counter1 / goal->num_measurements);
                goal_handle->publish_feedback(feedback);
            }
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "move the hawk to pose two then press enter to collect measurements");
        std::cin.get();
        RCLCPP_INFO(this->get_logger(), "collecting measurements at pose two");

        while (rclcpp::ok() && counter2 < goal->num_measurements)
        {
            if (goal_handle->is_canceling()) {
                result->success = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal canceled");
                return;
            }
            rclcpp::Time tic = global_.header.stamp;
            if (tic > toc)
            {
                meas2.header.stamp = tic;
                meas2.transform = mess2_plugins::update_measurement(meas2.transform, global_.transform, counter2);
                counter2 = counter2 + 1;
                toc = tic;
                
                feedback->progress = 0.5 * (counter2 / goal->num_measurements);
                goal_handle->publish_feedback(feedback);
            }
            rate.sleep();
        }

        auto quat_diff = mess2_plugins::get_vicon_calibration(meas1.transform, meas2.transform);
        if (rclcpp::ok()) {
            result->quat_diff = quat_diff;
            result->success = 1;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    }

    void _vicon_callback(const TransformStamped::SharedPtr msg)
    {
        global_.header = msg->header;
        global_.transform = msg->transform;
    }
};  
}  

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_actions::UAVCalibrationServer)