
#include <vector>
#include <Eigen/Dense>

struct TimeDomainMetrics {
    double Mp_percent;
    double e_ss;
    double t_s;
};

TimeDomainMetrics computeTimeDomainMetrics(
        const std::vector<double>& t,
        const std::vector<double>& e_scalar,
        double eps = 0.02, double r_step = 1.0)
{
    const double band = eps * std::abs(r_step);
    double Mp = 0.0;
    for (double e_val : e_scalar) {
        double abs_e = std::abs(e_val);
        if (abs_e > Mp) Mp = abs_e;
    }
    Mp = Mp / std::abs(r_step) * 100.0;

    double e_ss = e_scalar.back();

    int settled_idx = -1;
    for (size_t k = 0; k < e_scalar.size(); ++k) {
        bool all_within = true;
        for (size_t j = k; j < e_scalar.size(); ++j) {
            if (std::abs(e_scalar[j]) > band) {
                all_within = false;
                break;
            }
        }
        if (all_within) {
            settled_idx = static_cast<int>(k);
            break;
        }
    }
    double t_s = (settled_idx >= 0) ? t[settled_idx] : std::numeric_limits<double>::infinity();

    return {Mp, e_ss, t_s};
}

double trapz(const std::vector<double>& t,
             const std::vector<double>& y)
{
    double acc = 0.0;
    for (size_t k = 1; k < t.size(); ++k) {
        double dt = t[k] - t[k-1];
        acc += 0.5 * dt * (y[k] + y[k-1]);
    }
    return acc;
}

double computeISE(const std::vector<double>& t,
                  const std::vector<Eigen::VectorXd>& e)
{
    std::vector<double> e_norm2(e.size());
    for (size_t k = 0; k < e.size(); ++k) {
        e_norm2[k] = e[k].squaredNorm();
    }
    return trapz(t, e_norm2);
}
