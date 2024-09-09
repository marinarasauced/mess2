
#include "mess2_algorithm_plugins/cost.hpp"

using Constraints = mess2_algorithm_msgs::msg::Constraints;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Threat = mess2_algorithm_msgs::msg::ThreatField;

namespace mess2_algorithms
{
    std::pair<double, double> get_cost(const Graph& graph, const Threat& threat, Actor& actor, const int64_t index_parent_curr, const int64_t index_child_curr, const int64_t& index_parent_last, const std::vector<Constraints>& constraint)
    {
        // each element can be zero or non zero depending on the logic from the child, parent, and grandparent vertices
        auto time_to_wait = actor.get_time_to_wait(index_parent_curr, index_child_curr);
        auto time_to_rotate = actor.get_time_to_rotate(graph.vertices[index_parent_curr], graph.vertices[index_child_curr], graph.vertices[index_parent_last]);
        auto time_to_translate = actor.get_time_to_translate(graph.vertices[index_parent_curr], graph.vertices[index_child_curr]);

        // assume cost equals product of time at vertex and occupied threat at vertex
        auto threat_at_parent_curr = actor.retrieve_occupied_threat_at_vertex(threat, index_parent_curr);
        auto threat_at_child_curr = actor.retrieve_occupied_threat_at_vertex(threat, index_child_curr);

        auto cost_to_wait = time_to_wait * threat_at_parent_curr;
        auto cost_to_rotate = time_to_rotate * threat_at_parent_curr;
        auto cost_to_translate = time_to_translate * threat_at_child_curr;

        auto time = time_to_wait + time_to_rotate + time_to_translate;
        auto cost = cost_to_wait + cost_to_rotate + cost_to_translate;
        return std::pair<double, double>(cost, time);
    }

} // namespace mess2_algorithms
