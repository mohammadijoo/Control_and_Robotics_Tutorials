
#include <vector>
#include <cstddef>
#include <Eigen/Dense>

double computeISE(const std::vector<double>& t,
                  const std::vector<Eigen::VectorXd>& e)
{
    const std::size_t N = t.size();
    if (N < 2 || e.size() != N) {
        throw std::runtime_error("Invalid log lengths");
    }
    double J = 0.0;
    for (std::size_t k = 1; k < N; ++k) {
        const double dt = t[k] - t[k - 1];
        J += e[k - 1].squaredNorm() * dt;
    }
    return J;
}

Eigen::VectorXd computeRMS(const std::vector<double>& t,
                           const std::vector<Eigen::VectorXd>& e)
{
    const std::size_t N = t.size();
    const std::size_t n = static_cast<std::size_t>(e.front().size());
    Eigen::VectorXd sq_int = Eigen::VectorXd::Zero(n);

    double T = 0.0;
    for (std::size_t k = 1; k < N; ++k) {
        const double dt = t[k] - t[k - 1];
        sq_int += dt * e[k - 1].cwiseProduct(e[k - 1]);
        T += dt;
    }
    Eigen::VectorXd rms = sq_int;
    for (std::size_t i = 0; i < n; ++i) {
        rms[i] = std::sqrt(rms[i] / T);
    }
    return rms;
}
