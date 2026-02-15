#include <Eigen/Dense>
#include <cmath>
#include <cstddef>

struct ValidationMetricsCpp {
    double rmse;
    double vaf;
    double fit_percent;
    double r2;
    double aic;
    double bic;
};

ValidationMetricsCpp computeMetricsCpp(const Eigen::MatrixXd& tau_meas,
                                       const Eigen::MatrixXd& tau_pred,
                                       std::size_t p_params)
{
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const std::ptrdiff_t N = tau_meas.rows();
    const std::ptrdiff_t m = tau_meas.cols();
    MatrixXd e = tau_meas - tau_pred;

    // RMSE (flattened)
    double rmse = std::sqrt((e.array().square().rowwise().sum()).mean());

    // Flatten to a single vector
    VectorXd y = Eigen::Map<const VectorXd>(tau_meas.data(), N * m);
    VectorXd y_hat = Eigen::Map<const VectorXd>(tau_pred.data(), N * m);
    VectorXd e_flat = y - y_hat;

    double y_mean = y.mean();
    double e_mean = e_flat.mean();

    VectorXd y0 = y.array() - y_mean;
    VectorXd e0 = e_flat.array() - e_mean;

    double var_y = (y0.array().square().sum()) / (y.size() - 1);
    double var_e = (e0.array().square().sum()) / (e_flat.size() - 1);

    double vaf = 100.0 * (1.0 - var_e / var_y);

    double num = (y - y_hat).norm();
    double den = (y.array() - y_mean).matrix().norm();
    double fit_percent = 100.0 * (1.0 - num / den);

    double tss = (y.array() - y_mean).square().sum();
    double rss = (y - y_hat).squaredNorm();
    double r2 = 1.0 - rss / tss;

    double sigma2_hat = e_flat.squaredNorm() / static_cast<double>(N * m);
    double Nm = static_cast<double>(N * m);
    double aic = 2.0 * static_cast<double>(p_params) + Nm * std::log(sigma2_hat);
    double bic = static_cast<double>(p_params) * std::log(Nm) + Nm * std::log(sigma2_hat);

    ValidationMetricsCpp out { rmse, vaf, fit_percent, r2, aic, bic };
    return out;
}
      
