
#include "mess2_algorithm_plugins/actor.hpp"

using Graph = mess2_algorithm_msgs::msg::Graph;
using Occupancy = mess2_algorithm_msgs::msg::Occupancy;
using Threat = mess2_algorithm_msgs::msg::ThreatField;
using Vertex = mess2_algorithm_msgs::msg::Vertex;

double counter = 0;

namespace mess2_algorithms
{
    Actor::Actor(const std::string& actor_name, const std::string& actor_dir) : actor_name(actor_name)
    {
        (void) load_config(actor_dir);
        (void) load_specifications(actor_dir);
    }

    std::string Actor::get_actor_name() const { return actor_name; }
    std::string Actor::get_turtlebot3_model() const { return turtlebot3_model; }
    std::string Actor::get_lds_model() const { return lds_model; }
    double Actor::get_k_lin() const { return k_lin; }
    double Actor::get_k_ang() const { return k_ang; }
    double Actor::get_u_lin_max() const { return u_lin_max; }
    double Actor::get_u_ang_max() const { return u_ang_max; }
    double Actor::get_radius() const { return radius; }
    double Actor::get_x_lin_tol() const { return x_lin_tol; }
    double Actor::get_x_ang_tol() const { return x_ang_tol; }

    void Actor::load_config(const std::string& actor_dir)
    {
        auto config_path = actor_dir + "/" + actor_name + "/config.yaml";
        auto config = YAML::LoadFile(config_path);
        auto config_actor = config["actor"];

        actor_name = config_actor["name"].as<std::string>("actor");
        turtlebot3_model = config_actor["turtlebot3_model"].as<std::string>("burger");
        lds_model = config_actor["lds_model"].as<std::string>("LDS-02");
        k_lin = std::stod(config_actor["k_lin"].as<std::string>("1.0"));
        k_ang = std::stod(config_actor["k_ang"].as<std::string>("1.0"));
        u_ratio = std::stod(config_actor["u_ratio"].as<std::string>("0.7"));
        r_ratio = std::stod(config_actor["r_ratio"].as<std::string>("1.1"));
        x_lin_tol = std::stod(config_actor["x_lin_tol"].as<std::string>("0.01"));
        x_ang_tol = std::stod(config_actor["x_ang_tol"].as<std::string>("0.01"));
    }

    void Actor::load_specifications(const std::string& actor_dir)
    {
        auto specifications_path = actor_dir + "/" + turtlebot3_model + ".yaml";
        auto model = YAML::LoadFile(specifications_path);
        auto model_specifications = model["specifications"];

        radius = std::stod(model_specifications["radius"].as<std::string>("0.105")) * r_ratio;
        u_lin_max = std::stod(model_specifications["u_lin_max"].as<std::string>("0.22")) * u_ratio;
        u_ang_max = std::stod(model_specifications["u_ang_max"].as<std::string>("2.84")) * u_ratio;
    }

    double Actor::get_time_to_wait(const int64_t& index_parent, const int64_t& index_child)
    {
        if (index_child == index_parent){
            return 1.0;
        } else {
            return 0.0;
        }
    }

    double Actor::get_time_to_rotate(const Vertex& vertex_parent, const Vertex& vertex_child, const Vertex& vertex_grandparent)
    {
        if (vertex_parent.x == vertex_child.x && vertex_parent.y == vertex_child.y){
            return 0.0;
        }

        auto dx_gp2p = vertex_grandparent.x - vertex_parent.x;
        auto dy_gp2p = vertex_grandparent.y - vertex_parent.y;
        auto theta_gp2p = std::atan2(dy_gp2p, dx_gp2p);

        auto dx_p2c = vertex_parent.x - vertex_child.x;
        auto dy_p2c = vertex_parent.y - vertex_child.y;
        auto theta_p2c = std::atan2(dy_p2c, dx_p2c);

        auto theta = theta_p2c - theta_gp2p;
        if (theta > M_PI){
            theta -= 2 * M_PI;
        } else if (theta < -M_PI){
            theta += 2 * M_PI;
        }

        theta = std::abs(theta);

        if (theta == 0.0){
            return 0.0;
        }

        auto theta2 = theta;
        auto theta1 = 0.0;
        if (theta * k_ang > u_ang_max){
            theta2 = u_ang_max / k_ang;
            theta1 = theta - theta2;
        }

        auto time1 = theta1 / u_ang_max;
        auto time2 = -std::log(x_ang_tol / theta2) / k_ang;
        auto time = time1 + time2;
        return time;
    }

    double Actor::get_time_to_translate(const Vertex& vertex_parent, const Vertex& vertex_child)
    {
        auto dx_p2c = vertex_parent.x - vertex_child.x;
        auto dy_p2c = vertex_parent.y - vertex_child.y;
        auto distance_p2c = std::sqrt(dx_p2c * dx_p2c + dy_p2c * dy_p2c);

        auto time = distance_p2c / u_lin_max;
        return time; // lazy assumption that initial 0.01 rad rotational error is negligible
    }

    void Actor::fill_occupancies_by_vertex(const Graph& graph)
    {
        occupancies_by_vertex.resize(graph.vertices.size());
        for (std::vector<Vertex>::size_type iter = 0; iter < graph.vertices.size(); ++iter)
        {
            Occupancy occupancy;
            occupancy.occupied.resize(graph.vertices.size());

            const auto vertex_parent = graph.vertices[iter];
            for (std::vector<Vertex>::size_type jter = 0; jter < graph.vertices.size(); ++jter)
            {
                const auto vertex_child = graph.vertices[jter];
                const auto distance = std::sqrt(
                    std::pow(vertex_parent.x - vertex_child.x, 2) + 
                    std::pow(vertex_parent.y - vertex_child.y, 2)
                );
                if (distance <= radius){
                    occupancy.occupied[jter] = 1;
                } else {
                    occupancy.occupied[jter] = 0;
                }
            }
            occupancies_by_vertex[iter] = occupancy;
        }
    }

    Occupancy Actor::retrieve_occupancies_at_vertex(const int64_t& index_vertex)
    {
        auto occupancies = occupancies_by_vertex[index_vertex];
        return occupancies;
    }

    double Actor::retrieve_occupied_threat_at_vertex(const Threat& threat, const int64_t& index_vertex)
    {
        double threat_cummulative = 0.0;
        auto occupancies = retrieve_occupancies_at_vertex(index_vertex);

        for (std::vector<double>::size_type iter = 0; iter < threat.threat.size(); ++iter)
        {
            auto threat_value = threat.threat[iter];
            auto occupancy_bool = occupancies.occupied[iter];
            threat_cummulative += threat_value * occupancy_bool;
        }
        return threat_cummulative;
    }

} // namespace mess2_algorithms
