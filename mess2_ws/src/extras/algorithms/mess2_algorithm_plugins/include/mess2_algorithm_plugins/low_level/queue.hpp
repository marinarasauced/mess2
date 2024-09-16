#ifndef MESS2_ALGORITHM_PLUGINS_QUEUE_LL_HPP
#define MESS2_ALGORITHM_PLUGINS_QUEUE_LL_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    struct queue_ll {
        double score;
        double time;
        int64_t index_parent;
        int64_t index_history;
    };

    struct queue_ll_comparator {
        bool operator()(const queue_ll& a, const queue_ll& b) {
            if (a.score != b.score) {
                return a.score < b.score;
            }
            return a.time > b.time;
        }
    };

    class QueueLL
    {
    public:
        QueueLL();

        void add_queue(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history);
        queue_ll get_queue();
        void pop_queue();
    
    private:
        std::priority_queue<queue_ll, std::vector<queue_ll>, queue_ll_comparator> queue_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_QUEUE_LL_HPP