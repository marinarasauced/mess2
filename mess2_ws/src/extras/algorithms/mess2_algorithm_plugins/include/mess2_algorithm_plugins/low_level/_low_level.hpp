#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/_graph.hpp"
#include "mess2_algorithm_plugins/graph/edge.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"

namespace mess2_algorithms
{
    struct adjacency {
        // int64_t index_parent;
        std::vector<int64_t> childen;
    };

    class Adjacency {
    public:
        Adjacency();
        
        adjacency get_adjacencies(const int64_t& index_parent);

    private:
        std::vector<adjacency> adjacency_;
    };

    

    

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP