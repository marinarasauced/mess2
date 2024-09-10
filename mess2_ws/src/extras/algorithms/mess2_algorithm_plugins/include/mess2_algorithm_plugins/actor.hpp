#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

#include <yaml-cpp/yaml.h>

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
#include "mess2_algorithm_msgs/msg/occupancy.hpp"
#include "mess2_algorithm_msgs/msg/threat_field.hpp"
#include "mess2_algorithm_msgs/msg/vertex.hpp"

using Graph = mess2_algorithm_msgs::msg::Graph;
using Occupancy = mess2_algorithm_msgs::msg::Occupancy;
using Threat = mess2_algorithm_msgs::msg::ThreatField;
using Vertex = mess2_algorithm_msgs::msg::Vertex;

namespace mess2_algorithms
{
class Actor
{
public:
    Actor(const std::string& actor_name, const std::string& actor_dir, const Graph& graph, const Threat& threat);

    std::string get_actor_name() const;
    std::string get_turtlebot3_model() const;
    std::string get_lds_model() const;
    double get_k_lin() const;
    double get_k_ang() const;
    double get_u_lin_max() const;
    double get_u_ang_max() const;
    double get_radius() const;
    double get_x_lin_tol() const;
    double get_x_ang_tol() const;

    double get_time_to_wait(const int64_t& index_parent, const int64_t& index_child);
    double get_time_to_rotate(const Vertex& vertex_parent, const Vertex& vertex_child, const Vertex& vertex_grandparent);
    double get_time_to_translate(const Vertex& vertex_parent, const Vertex& vertex_child);

    void fill_occupancies_by_vertex(const Graph& graph);
    Occupancy retrieve_occupancies_at_vertex(const int64_t& index_vertex);

    void fill_threat_by_vertex(const Threat& threat);
    double retrieve_threat_at_vertex(const int64_t& index_vertex);

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
    double u_lin_max;
    double u_ang_max;
    double radius;
    double x_lin_tol;
    double x_ang_tol;
    std::vector<Occupancy> occupancies_by_vertex;
    std::vector<double> threat_by_vertex;
};

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
