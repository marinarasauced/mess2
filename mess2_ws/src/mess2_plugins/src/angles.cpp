#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mess2_msgs/msg/vertex.hpp"

namespace mess2_plugins {

    double wrap_to_pi(double angle) {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle <= -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    double get_angle_between_edges(mess2_msgs::msg::Vertex vertex0, mess2_msgs::msg::Vertex vertex1, mess2_msgs::msg::Vertex vertex2)
    {
        //
        double dx1_10 = vertex1.x1 - vertex0.x1;
        double dx2_10 = vertex1.x2 - vertex0.x2;
        double dx1_21 = vertex2.x1 - vertex1.x1;
        double dx2_21 = vertex2.x2 - vertex1.x2;

        // dot product and magnitudes
        double dot = dx1_10 * dx1_21 + dx2_10 * dx2_21;
        double mag_10 = std::sqrt(dx1_10 * dx1_10 + dx2_10 * dx2_10);
        double mag_21 = std::sqrt(dx1_21 * dx1_21 + dx2_21 * dx2_21);

        // cos(theta)
        double theta_inv = dot / (mag_10 * mag_21);
        theta_inv = std::max(-1.0, std::min(1.0, theta_inv));

        // theta
        double theta = std::acos(theta_inv);
        return theta;
    }

    /*
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
    } */
}