#ifndef MESS2_ALGORITHM_PLUGINS_THREAT_HPP
#define MESS2_ALGORITHM_PLUGINS_THREAT_HPP

#include </usr/include/armadillo>

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

#include "mess2_algorithm_msgs/msg/threat_field.hpp"

using Threat = mess2_algorithm_msgs::msg::ThreatField;

namespace mess2_algorithms
{
    Threat generate_threat(const arma::mat& x_mesh, const arma::mat& y_mesh);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_THREAT_HPP
