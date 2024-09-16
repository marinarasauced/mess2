#ifndef MESS2_ALGORITHM_PLUGINS_HISTORY_LL_HPP
#define MESS2_ALGORITHM_PLUGINS_HISTORY_LL_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    struct history_ll {
        double score;
        double time;
        int64_t index_parent;
        int64_t index_history;
    };

    class HistoryLL
    {
    public:
        HistoryLL();

        void add_history(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history);
        history_ll get_history(const int64_t& index_history);
    
    private:
        std::vector<history_ll> history_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_HISTORY_LL_HPP