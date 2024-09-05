#ifndef MESS2_ALGORITHM_PLUGINS_UTILS_HPP
#define MESS2_ALGORITHM_PLUGINS_UTILS_HPP

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
#include </usr/include/armadillo>

namespace mess2_algorithms
{
std::tuple<arma::mat, arma::mat> get_mesh(const double& x_min, const double& x_max, const double& y_min, const double& y_max, const int64_t& resolution);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_UTILS_HPP
