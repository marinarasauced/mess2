
#include "mess2_algorithm_plugins/low_level/queue.hpp"

namespace mess2_algorithms
{
    QueueLL::QueueLL() {};

    void QueueLL::add_queue(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history) {
        std::pair<double,double> scores = {score, time};
        queue_.emplace(queue_ll{score, time, index_parent, index_history});
    }

    queue_ll QueueLL::get_queue() {
        if (queue_.empty()) {
            throw std::runtime_error("attempt to get from an empty queue.");
        }
        return queue_.top();
    }

    void QueueLL::pop_queue() {
        queue_.pop();
    }

} // namespace mess2_algorithms
