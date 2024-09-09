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
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
    std::pair<double, double> get_cost(const Graph& graph, const Threat& threat, Actor& actor, const int64_t index_parent_curr, const int64_t index_child_curr, const int64_t& index_parent_last);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_COST_HPP
