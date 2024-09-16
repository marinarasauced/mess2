
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"

namespace mess2_algorithms
{
    Vertex::Vertex(const double& x, const double&y, const double& theta)
    {
        x_ = x;
        y_ = y;
        theta_ = theta;
    }

    double Vertex::get_x_() const {
        return x_;
    }

    double Vertex::get_y_() const {
        return y_;
    }

    double Vertex::get_theta_() const {
        return theta_;
    }

} // namespace mess2_algorithms
