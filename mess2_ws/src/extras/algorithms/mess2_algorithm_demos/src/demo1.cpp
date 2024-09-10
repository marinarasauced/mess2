
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

#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/cost.hpp"
#include "mess2_algorithm_plugins/graph.hpp"
#include "mess2_algorithm_plugins/low_level.hpp"
#include "mess2_algorithm_plugins/high_level.hpp"
#include "mess2_algorithm_plugins/threat.hpp"
#include "mess2_algorithm_plugins/utils.hpp"

using namespace mess2_algorithms;

class AlgorithmDemo1 : public rclcpp::Node
{
public:
    AlgorithmDemo1() : Node("demo1")
    {
        // graph generation
        bool diagonals = false;
        auto [x_graph, y_graph] = get_mesh(-3.0, 3.0, -3.0, 3.0, resolution_);
        auto graph = generate_graph(x_graph, y_graph, diagonals);
        graph.type = "directed";

        // threat generation
        auto [x_weights, y_weights] = get_mesh(-15.0, 15.0, -15.0, 15.0, resolution_);
        auto threat = generate_threat(x_weights, y_weights);

        // list of actors generation
        auto burger1 = Actor("burger1", actor_dir_, graph, threat);

        std::vector<Actor> actors;
        actors.push_back(burger1);

        // source index generation
        std::vector<int64_t> indices_source;
        indices_source.emplace_back(get_index_vertex_from_position(graph.vertices, -3.0, -3.0));

        // target index generation
        std::vector<int64_t> indices_target;
        indices_target.emplace_back(get_index_vertex_from_position(graph.vertices, 3.0, 3.0));

        // run algorithm
        std::vector<Constraints> constraints;
        constraints.resize(graph.vertices.size());
        auto path = execute_low_level_search(graph, threat, actors[0], indices_source[0], indices_target[0], constraints);
        // execute_high_level_search(graph, threat, actors, indices_source, indices_target);
    }

private:
    // hard-coded parameters
    std::string actor_dir_ = "/home/marinarasauced/Projets/mess2/actors/ugv";
    int64_t resolution_ = 100;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlgorithmDemo1>();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
