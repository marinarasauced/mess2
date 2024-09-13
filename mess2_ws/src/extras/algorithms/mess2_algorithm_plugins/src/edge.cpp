
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/edge.hpp"

namespace mess2_algorithms
{
    Edge::Edge(const int64_t& index_parent, const int64_t& index_child)
    {
        index_parent_ = index_parent;
        index_child_ = index_child;
    }

    int64_t Edge::get_index_parent_() const {
        return index_parent_;
    }

    int64_t Edge::get_index_child_() const {
        return index_child_;
    }

} // namespace mess2_algorithms
