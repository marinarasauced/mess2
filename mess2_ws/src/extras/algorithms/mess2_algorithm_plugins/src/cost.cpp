
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/cost.hpp"

namespace mess2_algorithms
{
double get_time_to_rotate()
{
    // state space model:
    // x_       = [theta - theta0]   ε  abs(theta - theta0)     <=  pi
    // x_dot    = [theta_dot]
    // u_       = theta_dot * k_ang  ε  abs(theta_dot * k_ang)  <=  u_ang_max,
    //          = u_ang_max          ε  abs(theta_dot * k_ang)  >   u_ang_max
    return 0.0;
}

double get_time_to_translate(const mess2_algorithm_msgs::msg::Vertex& source, const mess2_algorithm_msgs::msg::Vertex& target, const double& speed)
{
    // state space model:
    // x_       = [y - y0; theta - theta0]
    // x_dot    = [y_dot; theta_dot]
    // u_       = theta_dot

    auto distance = std::sqrt(
        std::pow(source.x - target.x, 2) + 
        std::pow(source.y - target.y, 2)
    );
    auto time = distance / speed;
    return time;
}

double get_time_to_wait()
{
    return 1.0;
}

double get_threat_by_occupancies(const mess2_algorithm_msgs::msg::ThreatField& weights, const int64_t& index, UGVActor& actor)
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

double get_cost_to_wait(const double& time, const double& threat)
{
    return threat * time;
}

double get_cost_to_rotate(const double& time, const double& threat)
{
    return threat * time;
}

double get_cost_to_translate(const double& time, const double& threat)
{
    return threat * time;
}

std::pair<double, double> cost(const mess2_algorithm_msgs::msg::Graph& graph, const mess2_algorithm_msgs::msg::ThreatField& weights, const double& current_weight, const double& current_time, const int64_t& source, const int64_t& target, const int64_t previous, UGVActor& actor)
{
    auto source_threat = get_threat_by_occupancies(weights, source, actor);
    auto target_threat = get_threat_by_occupancies(weights, target, actor);

    auto time_to_wait = get_time_to_wait();
    auto time_to_rotate = get_time_to_rotate();
    auto time_to_translate = get_time_to_translate(graph.vertices[source], graph.vertices[target], actor.get_u_lin_max());

    auto cost_to_wait = get_cost_to_wait(time_to_wait, source_threat);
    auto cost_to_rotate = get_cost_to_rotate(time_to_rotate, source_threat);
    auto cost_to_translate = get_cost_to_translate(time_to_translate, target_threat);

    if (source == target){
        auto future_time = time_to_wait;
        auto future_weight = cost_to_wait;
        std::pair<double, double> future_cost = {future_weight + current_weight, future_time + current_time};
        return future_cost;
    } else {
        auto future_time = time_to_rotate + time_to_translate;
        auto future_weight = cost_to_rotate + cost_to_translate;
        std::pair<double, double> future_cost = {future_weight + current_weight, future_time + current_time};
        return future_cost;
    }
}

} // namespace mess2_algorithms
