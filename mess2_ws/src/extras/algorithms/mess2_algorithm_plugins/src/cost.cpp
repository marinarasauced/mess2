
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/cost.hpp"

namespace mess2_algorithms
{
double get_threat_by_occupancies(const mess2_algorithm_msgs::msg::ThreatField& weights, const int64_t& index, Actor& actor)
{
    double threat = 0.0;
    auto occupied = actor.get_occupancies_by_vertex()[index];
    for (std::vector<int64_t>::size_type iter = 0; iter < occupied.occupied.size(); ++iter)
    {
        if (occupied.occupied[iter] == 1){
            threat = threat + weights.threat[iter];
        }
    }
    return threat;
}

double get_time_to_wait(const int64_t& source, const int64_t& target)
{
    if (source == target){
        return 1.0;
    } else {
        return 0.0;
    }
}

double get_time_to_rotate(const mess2_algorithm_msgs::msg::Vertex& source, const mess2_algorithm_msgs::msg::Vertex& target, const mess2_algorithm_msgs::msg::Vertex& previous, Actor& actor)
{
    // state space model:
    // x_       = [theta - theta0]   ε  abs(theta - theta0)     <=  pi
    // x_dot    = [theta_dot]
    // u_ang    = theta_dot * k_ang  ε  abs(theta_dot * k_ang)  <=  u_ang_max,
    //          = u_ang_max          ε  abs(theta_dot * k_ang)  >   u_ang_max

    // auto dx_p2s = source.x - previous.x;
    // auto dy_p2s = source.y - previous.y;
    // auto dx_s2t = target.x - source.x;
    // auto dy_s2t = target.y - source.y;

    // auto theta_p2s = std::atan2(dy_p2s, dx_p2s);
    // auto theta_s2t = std::atan2(dy_s2t, dx_s2t);

    return 0.0;
}

double get_time_to_translate(const mess2_algorithm_msgs::msg::Vertex& source, const mess2_algorithm_msgs::msg::Vertex& target, Actor& actor)
{
    // state space model:
    // x_       = [y - y0; theta - theta0]
    // x_dot    = [y_dot; theta_dot]
    // u_lin    = theta_dot (INCORRECT)

    auto u_lin_max = actor.get_u_lin_max();

    auto distance = std::sqrt(
        std::pow(source.x - target.x, 2) + 
        std::pow(source.y - target.y, 2)
    );
    auto time = distance / u_lin_max;
    return time;
}

std::pair<double, double> get_cost(const mess2_algorithm_msgs::msg::Graph& graph, const mess2_algorithm_msgs::msg::ThreatField& weights, const double& current_threat, const double& current_time, const int64_t& source, const int64_t& target, const int64_t previous, Actor& actor)
{
    auto threat_at_source = get_threat_by_occupancies(weights, source, actor);
    auto threat_at_target = get_threat_by_occupancies(weights, target, actor);

    auto time_to_wait = get_time_to_wait(source, target);
    auto time_to_rotate = get_time_to_rotate(graph.vertices[source], graph.vertices[target], graph.vertices[previous], actor);
    auto time_to_translate = get_time_to_translate(graph.vertices[source], graph.vertices[target], actor);

    auto time_at_source = time_to_wait + time_to_rotate;
    auto time_at_target = time_to_translate;

    auto future_time = time_at_source + time_at_target;
    auto future_cost = threat_at_source * time_at_source + threat_at_target * time_at_target;
    std::pair<double, double> future = {future_cost, future_time};
    return future;
}

} // namespace mess2_algorithms
