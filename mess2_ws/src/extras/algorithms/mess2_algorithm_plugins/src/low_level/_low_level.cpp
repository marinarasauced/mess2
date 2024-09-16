
#include "mess2_algorithm_plugins/low_level.hpp"

namespace mess2_algorithms
{
    Adjacency::Adjacency() {};

    

    

    void execute_low_level_search(Graph& graph, const int64_t& index_source, const int64_t& index_vertex)
    {
        auto history_ = History();
        auto queue_ = Queue();

        history_.add_history(0.0, 0.0, index_source, -1);
        queue_.add_queue(0.0, 0.0, index_source, 0);
    }

} // namespace mess2_algorithms
