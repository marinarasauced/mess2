
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

#include "rclcpp/rclcpp.hpp"

#include "mess2_algorithm_msgs/msg/goal.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/graph.hpp"
#include "mess2_algorithm_plugins/threat.hpp"
#include "mess2_algorithm_plugins/utils.hpp"

#include "mess2_algorithm_plugins/low_level/search.hpp"

// #include "mess2_algorithm_plugins/actor.hpp"
// #include "mess2_algorithm_plugins/cost.hpp"
// #include "mess2_algorithm_plugins/low_level/search.hpp"

using namespace mess2_algorithms;

class AlgorithmDemo1 : public rclcpp::Node
{
public:
    AlgorithmDemo1() : Node("demo1")
    {
        this->declare_parameter("actor_dir", "/home/marinarasauced/Projets/mess2/actors/ugv");
        this->declare_parameter("actor_names", std::vector<std::string>({"burger1"}));
        this->declare_parameter("resolution", 11);
        
        this->get_parameter("actor_dir", actor_dir_);
        this->get_parameter("actor_names", actor_names_);
        this->get_parameter("resolution", resolution_);

        auto [x_graph, y_graph] = get_mesh(-3.0, 3.0, -3.0, 3.0, resolution_);
        auto graph = generate_graph(x_graph, y_graph);
        graph.type = "undirected";

        auto [x_weights, y_weights] = get_mesh(-15.0, 15.0, -15.0, 15.0, resolution_);
        auto weights = generate_threat(x_weights, y_weights);

        auto source = get_index_of_vertex(graph.vertices, -3.0, -3.0);
        auto target = get_index_of_vertex(graph.vertices, 3.0, 3.0);

        std::vector<Actor> actors;
        for (const std::string actor_name : actor_names_){
            actors.emplace_back(Actor(actor_name, actor_dir_));
        }

        for (Actor actor : actors)
        {
            pathplan
        }


        // auto graph

        // auto graph = 
        // auto actor1 = Actor("burger1", "home/marinarasauced/Projets/mess2/actors/ugv");
    }

private:







    double resolution_;
    std::vector<std::string> actor_names_;
    std::string actor_dir_;
};
