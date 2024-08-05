#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mess2_msgs/msg/edge.hpp"

namespace mess2_plugins {

    double wrap_to_pi(double angle) {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle <= -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    double get_angle_between_edges(mess2_msgs::msg::Edge last, mess2_msgs::msg::Edge curr)
    {
        // last and current edges
        double dx0 = last.x1[1] - last.x1[0];
        double dy0 = last.x2[1] - last.x2[0];
        double dx1 = curr.x1[1] - curr.x1[0];
        double dy1 = curr.x2[1] - curr.x2[0];

        // dot product
        double dot = dx0 * dx1 + dy0 * dy1;

        // magnitudes
        double mag0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
        double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

        // cos(theta)
        double cos_theta = dot / (mag0 * mag1);
        cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
        
        // theta
        double theta = std::acos(cos_theta);
        return theta;
    }
}