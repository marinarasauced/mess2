#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

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
#include <yaml-cpp/yaml.h>
#include </usr/include/armadillo>

#include "mess2_algorithm_msgs/msg/graph.hpp"
#include "mess2_algorithm_msgs/msg/occupancy.hpp"

namespace mess2_algorithms
{
class Actor
{
public:
    Actor(const std::string& actor_name, const std::string& actor_dir);

    std::string get_actor_name() const;
    std::string get_turtlebot3_model() const;
    std::string get_lds_model() const;
    double get_k_lin() const;
    double get_k_ang() const;
    double get_u_lin_max() const;
    double get_u_ang_max() const;
    double get_radius() const;

    std::vector<mess2_algorithm_msgs::msg::Occupancy> get_occupancies_by_vertex();
    std::vector<mess2_algorithm_msgs::msg::Occupancy> get_occupancies_by_edge();

    void fill_occupancies_by_vertex(const mess2_algorithm_msgs::msg::Graph& graph);
    void fill_occupancies_by_edge(const mess2_algorithm_msgs::msg::Graph& graph);

private:
    void load_config(const std::string& actor_dir);
    void load_specifications(const std::string& actor_dir);

    std::string actor_name;
    std::string turtlebot3_model;
    std::string lds_model;
    double k_lin;
    double k_ang;
    double u_ratio;
    double r_ratio;
    double radius;
    double u_lin_max;
    double u_ang_max;
    std::vector<mess2_algorithm_msgs::msg::Occupancy> occupancies_by_vertex;
    std::vector<mess2_algorithm_msgs::msg::Occupancy> occupancies_by_edge;
};

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
