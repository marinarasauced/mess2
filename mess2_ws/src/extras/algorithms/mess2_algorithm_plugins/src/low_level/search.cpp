
#include "mess2_algorithm_plugins/cost.hpp"
#include "mess2_algorithm_plugins/low_level/search.hpp"

namespace mess2_algorithms
{
mess2_algorithm_msgs::msg::Path pathplan(const mess2_algorithm_msgs::msg::Graph& graph, const mess2_algorithm_msgs::msg::ThreatField& weights, const int64_t& source, const int64_t& target, Actor& actor)
{
    auto num_vertices = static_cast<int64_t>(graph.vertices.size());
    // auto num_edges = static_cast<int64_t>(graph.edges.size());

    std::priority_queue<std::tuple<double, double, int64_t, int64_t>, std::vector<std::tuple<double, double, int64_t, int64_t>>, std::greater<>> queue;
    std::vector<std::tuple<double, double, int64_t, int64_t>> history;
    std::vector<std::vector<int64_t>> neighbors(num_vertices);

    for (const auto& edge : graph.edges)
    {
        if (graph.type == "directed"){
            neighbors[edge.source].emplace_back(edge.target);
        } else if (graph.type == "undirected"){
            if (edge.source == edge.target){
                neighbors[edge.source].emplace_back(edge.target);
            } else {
                neighbors[edge.source].emplace_back(edge.target);
                neighbors[edge.target].emplace_back(edge.source);
            }
        } else {
            neighbors[edge.source].emplace_back(edge.target);
        }
    }

    history.emplace_back(0.0, 0.0, source, -1);
    queue.emplace(0.0, 0.0, source, 0);

    while (!queue.empty())
    {
        auto [current_weight, current_time, parent, current_lineage] = queue.top();
        queue.pop();

        auto grand_parent = std::get<2>(history[current_lineage]);
        auto future_lineage = static_cast<int64_t>(history.size()) - 1;

        for (const auto& child : neighbors[parent])
        {
            future_lineage = future_lineage + 1;
            auto [future_weight, future_time] = get_cost(graph, weights, current_weight, current_time, child, parent, grand_parent, actor);

            history.emplace_back(future_weight, future_time, child, current_lineage);
            queue.emplace(future_weight, future_time, child, future_lineage);

            if (child == target) {
                break;
            }
        }
    }

    // starting at last heritage, work backwards
    mess2_algorithm_msgs::msg::Path path;
    if (!history.empty())
    {
        auto [weight, time, parent, lineage] = history.back();
        while (lineage != -1)
        {
            
        }
    }

}

} // namespace mess2_algorithms
