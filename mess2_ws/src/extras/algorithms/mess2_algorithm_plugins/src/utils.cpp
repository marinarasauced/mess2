
#include "mess2_algorithm_plugins/utils.hpp"

namespace mess2_algorithms
{
std::tuple<arma::mat, arma::mat> get_mesh(const double& x_min, const double& x_max, const double& y_min, const double& y_max, const int64_t& resolution)
{
    auto x_vector = arma::linspace(x_min, x_max, resolution);
    auto y_vector = arma::linspace(y_min, y_max, resolution);

    int n_rows = y_vector.n_elem;
    int n_cols = x_vector.n_elem;
    arma::mat x_mesh(n_rows, n_cols);
    arma::mat y_mesh(n_rows, n_cols);
    for (int iter = 0; iter < n_rows; ++iter)
    {
        x_mesh.row(iter) = x_vector.t();
        y_mesh.row(iter).fill(y_vector(iter));
    }
    return std::make_tuple(x_mesh, y_mesh);
}

} // namespace mess2_algorithms
