#ifndef MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_HPP
#define MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_HPP

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

#include "mess2_algorithm_msgs/msg/constraint.hpp"
#include "mess2_algorithm_msgs/msg/constraints.hpp"
#include "mess2_algorithm_msgs/msg/graph.hpp"
#include "mess2_algorithm_msgs/msg/occupancy.hpp"
#include "mess2_algorithm_msgs/msg/path.hpp"
#include "mess2_algorithm_msgs/msg/segment.hpp"
#include "mess2_algorithm_msgs/msg/threat_field.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/low_level.hpp"

using Graph = mess2_algorithm_msgs::msg::Graph;
using Threat = mess2_algorithm_msgs::msg::ThreatField;

namespace mess2_algorithms
{
    void execute_high_level_search(const Graph& graph, const Threat& threat, std::vector<Actor>& actors, const std::vector<int64_t>& indices_source, const std::vector<int64_t>& indices_target);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_HPP
