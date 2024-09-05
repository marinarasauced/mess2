#ifndef MESS2_ALGORITHM_PLUGINS_SEARCH_HPP
#define MESS2_ALGORITHM_PLUGINS_SEARCH_HPP

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
#include "mess2_algorithm_msgs/msg/path.hpp"
#include "mess2_algorithm_msgs/msg/segment.hpp"
#include "mess2_algorithm_msgs/msg/threat_field.hpp"
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
mess2_algorithm_msgs::msg::Path pathplan(const mess2_algorithm_msgs::msg::Graph& graph, const mess2_algorithm_msgs::msg::ThreatField& weights, const int64_t& source, const int64_t& target, Actor& actor);
}

#endif // MESS2_ALGORITHM_PLUGINS_SEARCH_HPP
