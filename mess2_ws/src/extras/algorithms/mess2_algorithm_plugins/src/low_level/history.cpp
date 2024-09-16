
#include "mess2_algorithm_plugins/low_level/history.hpp"

namespace mess2_algorithms
{
    HistoryLL::HistoryLL() {};

    void HistoryLL::add_history(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history) {
        history_.emplace_back(history_ll{score, time, index_parent, index_history});
    }

    history_ll HistoryLL::get_history(const int64_t& index_history) {
        if (index_history < 0 || index_history >= static_cast<int64_t>(history_.size())) {
            throw std::out_of_range("history index out of range in get_history.");
        }
        return history_[index_history];
    }

} // namespace mess2_algorithms
