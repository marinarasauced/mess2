
#include "mess2_algorithm_plugins/cost.hpp"
#include "mess2_algorithm_plugins/low_level/search.hpp"

using Graph = mess2_algorithm_msgs::msg::Graph;
using ThreatField = mess2_algorithm_msgs::msg::ThreatField;
using UGVActor = mess2_algorithms::UGVActor;

// cummulative weight, cummulative time, vertex, lineage
using AlgorithmRecord = std::tuple<double, double, int64_t, int64_t>;

namespace mess2_algorithms
{
void execute(const Graph& graph, const ThreatField& weights, const int64_t& source, const int64_t& target, UGVActor& actor)
{
    auto num_vertices = static_cast<int64_t>(graph.vertices.size());
    // auto num_edges = static_cast<int64_t>(graph.edges.size());

    std::priority_queue<AlgorithmRecord, std::vector<AlgorithmRecord>, std::greater<>> queue;
    std::vector<AlgorithmRecord> history;
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
        }
    }

    history.emplace_back(0.0, 0.0, source, -1);
    queue.emplace(0.0, 0.0, source, 0);

    while (!queue.empty())
    {
        auto [current_weight, current_time, parent, current_lineage] = queue.top();
        queue.pop();

        // auto [_, _, grand_parent, _] = history[current_lineage];
        auto grand_parent = std::get<2>(history[current_lineage]);
        auto future_lineage = static_cast<int64_t>(history.size()) - 1;

        for (const auto& child : neighbors[parent])
        {
            future_lineage = future_lineage + 1;
            auto [future_weight, future_time] = mess2_algorithms::cost(graph, weights, current_weight, current_time, child, parent, grand_parent, actor);

            history.emplace_back(future_weight, future_time, child, current_lineage);
            queue.emplace(future_weight, future_time, child, future_lineage);

            if (child == target) {
                break;
            }
        }

    }
}

} // namespace mess2_algorithms
