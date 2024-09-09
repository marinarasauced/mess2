
#include "mess2_algorithm_plugins/cost.hpp"

using Constraints = mess2_algorithm_msgs::msg::Constraints;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Threat = mess2_algorithm_msgs::msg::ThreatField;

namespace mess2_algorithms
{
    std::pair<double, double> get_cost(const Graph& graph, const Threat& threat, Actor& actor, const int64_t index_parent_curr, const int64_t index_child_curr, const int64_t& index_parent_last, const std::vector<Constraints>& constraints, const double& threat_curr, const double& time_curr)
    {
        // each element can be zero or non zero depending on the logic from the child, parent, and grandparent vertices
        auto time_to_wait = actor.get_time_to_wait(index_parent_curr, index_child_curr);
        auto time_to_rotate = actor.get_time_to_rotate(graph.vertices[index_parent_curr], graph.vertices[index_child_curr], graph.vertices[index_parent_last]);
        auto time_to_translate = actor.get_time_to_translate(graph.vertices[index_parent_curr], graph.vertices[index_child_curr]);
        auto time_next = time_curr + time_to_wait + time_to_rotate + time_to_translate;

        // get indices of all currently occupied vertices (i.e., where occupancy = 1)
        std::vector<int64_t> indices_occupancy;
        auto occupancies_at_child_curr = actor.retrieve_occupancies_at_vertex(index_child_curr);
        for (int64_t iter; iter < static_cast<int64_t>(occupancies_at_child_curr.occupied.size()); ++iter)
        {
            if (occupancies_at_child_curr.occupied[iter] == 1)
            {
                indices_occupancy.emplace_back(iter);
            }
        }

        //
        for (std::vector<int64_t>::size_type iter = 0; iter < indices_occupancy.size(); ++iter)
        {
            auto index_occupancy = indices_occupancy[iter];
            auto constraints_at_vertex = constraints[index_occupancy];
            for (auto& constraint_at_vertex : constraints_at_vertex.constraints)
            {
                auto time_curr_constrained = constraint_at_vertex.stamp[0];
                auto time_next_constrained = constraint_at_vertex.stamp[1];

                if (time_next < time_next_constrained && time_next > time_curr_constrained)
                {
                    auto threat_next = std::numeric_limits<double>::max();
                    return std::pair<double, double>(threat_next, time_next);
                }
            }
        }


        // assume cost equals product of time at vertex and occupied threat at vertex
        auto threat_at_parent_curr = actor.retrieve_occupied_threat_at_vertex(threat, index_parent_curr);
        auto threat_at_child_curr = actor.retrieve_occupied_threat_at_vertex(threat, index_child_curr);

        // cost from threat and time product
        auto cost_to_wait = time_to_wait * threat_at_parent_curr;
        auto cost_to_rotate = time_to_rotate * threat_at_parent_curr;
        auto cost_to_translate = time_to_translate * threat_at_child_curr;
        auto threat_next = threat_curr + cost_to_wait + cost_to_rotate + cost_to_translate;

        //
        return std::pair<double, double>(threat_next, time_next);
    }

} // namespace mess2_algorithms
