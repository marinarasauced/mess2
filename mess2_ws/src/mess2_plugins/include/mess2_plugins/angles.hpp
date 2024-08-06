#ifndef MESS2_PLUGINS__ANGLES_HPP_
#define MESS2_PLUGINS__ANGLES_HPP_

#include "mess2_msgs/msg/vertex.hpp"

namespace mess2_plugins {

double wrap_to_pi(double angle);

double get_angle_between_edges(mess2_msgs::msg::Vertex vertex0, mess2_msgs::msg::Vertex vertex1, mess2_msgs::msg::Vertex vertex2);

}  // namespace mess2_plugins

#endif  // MESS2_PLUGINS__ANGLES_HPP_