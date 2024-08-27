
#include "mess2_plugins/ugv.hpp"

namespace mess2_plugins {

    double get_max_u_lin(const std::string model, const double speed)
    {
        if (model == "burger")
        {
            return 0.22 * speed;
        }
        else if (model == "wafflepi")
        {
            return 0.26 * speed;
        }
        else
        {
            return 0.22 * speed;
        }
    }

    double get_max_u_ang(const std::string model, const double speed)
    {
        if (model == "burger")
        {
            return 2.84 * speed;
        }
        else if (model == "wafflepi")
        {
            return 1.82 * speed;
        }
        else
        {
            return 1.82 * speed;
        }
    }
}
