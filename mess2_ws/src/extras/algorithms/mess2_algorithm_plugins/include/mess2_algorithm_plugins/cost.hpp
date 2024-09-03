#ifndef MESS2_ALGORITHM_PLUGINS_COST_HPP
#define MESS2_ALGORITHM_PLUGINS_COST_HPP

#include <cmath>
#include <cstddef>
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

#include "mess2_algorithm_msgs/msg/graph.hpp"
#include "mess2_algorithm_msgs/msg/threat_field.hpp"
#include "mess2_algorithm_msgs/msg/vertex.hpp"
// #include "mess2_algorithm_msgs/msg/occupancy.hpp"
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
std::pair<double, double> cost(const mess2_algorithm_msgs::msg::Graph& graph, const mess2_algorithm_msgs::msg::ThreatField& weights, const double& current_weight, const double& current_time, const int64_t& child, const int64_t& parent, const int64_t grand_parent, UGVActor& actor);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_COST_HPP
