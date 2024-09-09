
#include "mess2_algorithm_plugins/high_level.hpp"

using Constraint = mess2_algorithm_msgs::msg::Constraint;
using Graph = mess2_algorithm_msgs::msg::Graph;
using Occupancy = mess2_algorithm_msgs::msg::Occupancy;
using Path = mess2_algorithm_msgs::msg::Path;
using Segment = mess2_algorithm_msgs::msg::Segment;
using Threat = mess2_algorithm_msgs::msg::ThreatField;

using Plan = std::vector<Path>;
using PlanSort = std::vector<std::tuple<double, double, int64_t, int64_t, int64_t>>;
using Queue = std::vector<std::vector<Constraint>>;

namespace mess2_algorithms
{
    void execute_high_level_search(const Graph& graph, const Threat& threat, std::vector<Actor>& actors, const std::vector<int64_t>& indices_source, const std::vector<int64_t>& indices_target)
    {
        // create an empty constraint array for each actor and push to queue
        Queue constraints;
        constraints.resize(actors.size());
        for (std::vector<Actor>::size_type iter = 0; iter < actors.size(); ++iter){
            constraints[iter].resize(graph.vertices.size());
        }
        std::queue<Queue> queue;
        queue.push(constraints);

        //
        bool conflict = false;
        while (!queue.empty() && conflict == true)
        {
            //
            auto constraints = queue.front();
            queue.pop();

            // run the low level search for each actor for the current constraints
            Plan plans;
            plans.resize(actors.size());
            for (std::vector<Actor>::size_type iter = 0; iter < actors.size(); ++iter)
            {
                auto index_source = indices_source[iter];
                auto index_target = indices_target[iter];
                std::vector<Constraint> constraint = constraints[iter];
                auto path = execute_low_level_search(
                    graph, threat, actors[iter], index_source, index_target, constraint
                );
                plans[iter] = path;
            }

            // create combined plan for all actors
            PlanSort plansort;
            for (std::vector<Actor>::size_type iter = 0; iter < actors.size(); ++iter)
            {
                auto path = plans[iter];
                for (std::vector<Segment>::size_type jter = 0; jter < path.segments.size(); ++jter)
                {
                    auto segment_path = path.segments[jter];
                    auto segment_plan = std::tuple<double, double, int64_t, int64_t, int64_t>(segment_path.stamp[0], segment_path.stamp[1], segment_path.segment.index_parent, segment_path.segment.index_child, iter);
                    plansort.push_back(segment_plan);
                }
            }

            // sort combined plan for all actors by time stamps
            std::sort(plansort.begin(), plansort.end(), [](const auto& a, const auto& b) {
                if (std::get<0>(a) == std::get<0>(b)) { return std::get<1>(a) < std::get<1>(b); }
                return std::get<0>(a) < std::get<0>(b);
            });

            // simulate the plans and check for conflicts
            //  - create empty occupancies for all actors
            std::vector<Occupancy> occupancies;
            occupancies.resize(actors.size());

            //  - fill occupancies for each actor
            for (std::vector<Occupancy>::size_type iter = 0; iter < actors.size(); ++iter){
                occupancies[iter].occupied.resize(graph.vertices.size(), 0);    
            }

            //  - run simulation of plans with estimated time steps and occupancy updating
            for (std::vector<std::tuple<double, double, int64_t, int64_t, int64_t>>::size_type iter; iter < plansort.size(); ++iter)
            {
                // retrieve current segment
                auto [time_curr, time_next, index_parent_curr, index_child_curr, index_actor_curr] = plansort[iter];

                // update occupancy for current agent at child
                occupancies[index_actor_curr] = actors[index_parent_curr].retrieve_occupancies_at_vertex(index_child_curr);

                // need to compare occupancies and determine if there is overlap
                std::vector<int8_t> occupied(graph.vertices.size(), 0);
                for (const auto& occupancy : occupancies)
                {
                    for (std::size_t jter = 0; jter < occupancy.occupied.size(); ++jter)
                    {
                        occupied[jter] += occupancy.occupied[jter];
                    }
                }

                // search vertices until first conflict is found
                for (std::size_t iter = 0; iter < occupied.size(); ++iter)
                {
                    if (occupied[iter] > 1)
                    {
                        conflict = true;

                        // retrieve indices of actors that occupy conflict vertex in time range
                        std::vector<int64_t> index_conflicting_actors;
                        for (std::vector<Actor>::size_type jter = 0; jter < actors.size(); ++jter)
                        {
                            if (occupancies[jter].occupied[iter] == 1)
                            {
                                index_conflicting_actors.emplace_back(static_cast<int64_t>(jter));
                            }
                        }

                        // now with list of actors that have conflict at conflicting vertex, generate constraints for queue
                        for (int64_t index_conflicting_actor : index_conflicting_actors)
                        {
                            auto constraints_new = constraints;
                            for (int64_t index_conflicting_other : index_conflicting_actors)
                            {
                                if (index_conflicting_other != index_conflicting_actor)
                                {
                                    Constraint constraint_new;
                                    constraint_new.stamp[0] = time_curr;
                                    constraint_new.stamp[1] = time_next;

                                    constraints_new[index_conflicting_other].push_back(constraint_new);
                                }
                            }
                            queue.push(constraints_new);
                        }

                        break;
                    }
                }

                // 

            }
        }

        // retrieve plans?
    }

} // namespace mess2_algorithms








// so we need a high level queue now
// requirements
// - each entry needs to specify constraints for each respective actor

/**
 * possible approaches:
 *  - each actor can have an attribute of all vertices, for each vertex, contain a list of time range stamps
 * 
 * how would this look in practice ?
 *  - in the low level:
 *      - you would calculate the time to transition to a vertex, then get a list of occupancies at the child vertex; if one of the occupancied vertices has a constraint in a time range containing the next time (curr time + time to transition), then set cost to infinite, thus killing the next entry in the search queue effectively
 *  - in the high level:
 *      - you have a list of actors, a queue where each element contains a constraints vector for each respective actor
 * 
 * first iteration:
 *      - add an entry to the queue with no constraints for all actors
 *      - run the low level search for all actors and store the paths
 *      - for each actor, retrieve the path indices and time stamps
 *          - in this case, the stamps will consider the time from parent to child
 *          - create a list of all paths marked with their actors and time stamps
 */