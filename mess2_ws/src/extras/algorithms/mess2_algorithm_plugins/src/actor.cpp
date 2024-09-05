
#include "mess2_algorithm_plugins/actor.hpp"

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

std::vector<mess2_algorithm_msgs::msg::Occupancy> Actor::get_occupancies_by_vertex() { return occupancies_by_vertex; }
std::vector<mess2_algorithm_msgs::msg::Occupancy> Actor::get_occupancies_by_edge()  { return occupancies_by_edge; }

void Actor::fill_occupancies_by_vertex(const mess2_algorithm_msgs::msg::Graph& graph)
{
    occupancies_by_vertex.resize(graph.vertices.size());
    for (std::vector<mess2_algorithm_msgs::msg::Vertex>::size_type iter = 0; iter < graph.vertices.size(); ++iter)
    {
        mess2_algorithm_msgs::msg::Occupancy occupancy;
        occupancy.occupied.resize(graph.vertices.size(), 0);

        const auto parent = graph.vertices[iter];
        for (std::vector<mess2_algorithm_msgs::msg::Vertex>::size_type jter = 0; jter < graph.vertices.size(); ++jter)
        {
            const auto child = graph.vertices[jter];
            const auto distance = std::sqrt(
                std::pow(parent.x - child.x, 2) + 
                std::pow(parent.y - child.y, 2)
            );
            if (distance <= radius){
                occupancy.occupied[jter] = 1;
            }
        }
        occupancies_by_vertex[iter] = occupancy;
    }
}

void Actor::fill_occupancies_by_edge(const mess2_algorithm_msgs::msg::Graph& graph)
{
    occupancies_by_edge.resize(graph.edges.size());
    for (std::vector<mess2_algorithm_msgs::msg::Edge>::size_type iter = 0; iter < graph.vertices.size(); ++iter)
    {
        mess2_algorithm_msgs::msg::Occupancy occupancy;
        occupancy.occupied.resize(graph.edges.size(), 0);

        const auto parent1 = graph.vertices[graph.edges[iter].source];
        const auto parent2 = graph.vertices[graph.edges[iter].target];
        for (std::vector<mess2_algorithm_msgs::msg::Vertex>::size_type jter = 0; jter < graph.vertices.size(); ++jter)
        {
            const auto child = graph.vertices[jter];
            const auto distance1 = std::sqrt(
                std::pow(parent1.x - child.x, 2) + 
                std::pow(parent1.y - child.y, 2)
            );
            const auto distance2 = std::sqrt(
                std::pow(parent2.x - child.x, 2) + 
                std::pow(parent2.y - child.y, 2)
            );
            if (distance1 <= radius || distance2 <= radius){
                occupancy.occupied[jter] = 1;
            }
        }
    }
}

} // namespace mess2_algorithms
