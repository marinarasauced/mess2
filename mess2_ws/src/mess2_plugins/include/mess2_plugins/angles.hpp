#ifndef MESS2_PLUGINS__ANGLES_HPP_
#define MESS2_PLUGINS__ANGLES_HPP_

#include "mess2_msgs/msg/edge.hpp"

namespace mess2_plugins {

double wrap_to_pi(double angle);

double get_angle_between_edges(mess2_msgs::msg::Edge last, mess2_msgs::msg::Edge curr);

}  // namespace mess2_plugins

#endif  // MESS2_PLUGINS__ANGLES_HPP_