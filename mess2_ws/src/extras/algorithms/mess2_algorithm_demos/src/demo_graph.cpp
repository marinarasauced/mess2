
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/edge.hpp"
#include "mess2_algorithm_plugins/graph.hpp"
#include "mess2_algorithm_plugins/utils.hpp"
#include "mess2_algorithm_plugins/vertex.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace mess2_algorithms;

class DemoGraph : public rclcpp::Node
{
public:
    DemoGraph() : Node("demo_graph")
    {
        auto [x_mesh, y_mesh] = get_mesh(-3.0, 3.0, -3.0, 3.0, 3);
        graph = generate_graph(x_mesh, y_mesh);
        graph.print_vertices();
        graph.print_edges();
    }

private:
    Graph graph;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoGraph>();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
