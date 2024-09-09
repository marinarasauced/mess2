
#include "mess2_algorithm_plugins/low_level.hpp"

using Constraints = mess2_algorithm_msgs::msg::Constraints;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Path = mess2_algorithm_msgs::msg::Path;
using Segment = mess2_algorithm_msgs::msg::Segment;
using Threat = mess2_algorithm_msgs::msg::ThreatField;

using History = std::tuple<double, double, int64_t, int64_t>;
using Queue = std::tuple<double, double, int64_t, int64_t>;

namespace mess2_algorithms
{
    Path execute_low_level_search(const Graph& graph, const Threat& threat, Actor& actor, int64_t& index_source, int64_t& index_target, const std::vector<Constraints>& constraint)
    {
        //
        std::vector<std::vector<int64_t>> adjacency(static_cast<int64_t>(graph.vertices.size()));
        std::vector<History> history;
        std::priority_queue<Queue, std::vector<Queue>, std::greater<>> queue;

        //
        for (const auto& edge : graph.edges)
        {
            if (graph.type == "directed"){
                adjacency[edge.index_parent].emplace_back(edge.index_child);
            } else if (graph.type == "undirected"){
                if (edge.index_parent == edge.index_child){
                    adjacency[edge.index_parent].emplace_back(edge.index_child);
                } else {
                    adjacency[edge.index_parent].emplace_back(edge.index_child);
                    adjacency[edge.index_child].emplace_back(edge.index_parent);
                }
            } else {
                adjacency[edge.index_parent].emplace_back(edge.index_child);
            }
        }

        //
        history.emplace_back(0.0, 0.0, index_source, -1);
        queue.emplace(0.0, 0.0, index_source, 0);

        //
        while (!queue.empty())
        {
            //
            auto [threat_curr, time_curr, index_parent_curr, index_history_curr] = queue.top();
            auto [threat_last, time_last, index_parent_last, index_history_last] = history[index_history_curr];
            queue.pop();

            //
            auto index_history_next = static_cast<int64_t>(history.size());
            for (const auto& index_child_curr : adjacency[index_parent_curr])
            {
                auto [threat_next, time_next] = get_cost(graph, threat, actor, index_parent_curr, index_child_curr, index_parent_last, constraint, threat_curr, time_curr);

                history.emplace_back(threat_next, time_next, index_child_curr, index_history_curr);
                queue.emplace(threat_next, time_next, index_child_curr, index_history_next);
                if (index_child_curr == index_target){ break; }
            }
        }

        // generate path in backwards order
        Path path;
        auto [threat_curr, time_curr, index_parent_curr, index_history_curr] = history.back();
        while (index_history_curr != -1)
        {
            auto [threat_last, time_last, index_parent_last, index_history_last] = history[index_history_curr];

            Segment segment;
            segment.segment.index_parent = index_parent_last;
            segment.segment.index_child = index_parent_curr;
            segment.stamp = {time_last, time_curr};
            path.segments.push_back(segment);

            threat_curr = threat_last;
            time_curr = time_last;
            index_parent_curr = index_parent_last;
            index_history_curr = index_history_last;
        }

        std::reverse(path.segments.begin(), path.segments.end());
        return path;
    }

} // namespace mess2_algorithms
