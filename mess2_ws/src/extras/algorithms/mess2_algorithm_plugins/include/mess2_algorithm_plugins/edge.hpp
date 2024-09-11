#ifndef MESS2_ALGORITHM_PLUGINS_EDGE_HPP
#define MESS2_ALGORITHM_PLUGINS_EDGE_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    class Edge
    {
    public:
        Edge(const int64_t& index_parent, const int64_t& index_child);
    private:
        int64_t index_parent_;
        int64_t index_child_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_EDGE_HPP