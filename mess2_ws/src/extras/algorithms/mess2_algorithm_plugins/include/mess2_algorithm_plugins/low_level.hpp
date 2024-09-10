#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP

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
#include "mess2_algorithm_msgs/msg/path.hpp"
#include "mess2_algorithm_msgs/msg/segment.hpp"
#include "mess2_algorithm_msgs/msg/threat_field.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/cost.hpp"

using Constraints = mess2_algorithm_msgs::msg::Constraints;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Path = mess2_algorithm_msgs::msg::Path;
using Threat = mess2_algorithm_msgs::msg::ThreatField;

using Adjacency = std::vector<std::vector<int64_t>>;
using History = std::tuple<double, double, int64_t, int64_t>;

namespace mess2_algorithms
{
    Adjacency generate_adjacency(const Graph& graph);

    Path retrieve_path(const std::vector<History>& history);

    Path execute_low_level_search(const Graph& graph, const Threat& threat, Actor& actor, int64_t& index_source, int64_t& index_target, const std::vector<Constraints>& constraint);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
