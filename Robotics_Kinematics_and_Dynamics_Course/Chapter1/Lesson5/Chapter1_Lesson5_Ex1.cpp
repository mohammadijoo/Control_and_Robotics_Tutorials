#include <Eigen/Dense>
#include <functional>

// f: R^n -> R^m, implemented as std::function<Eigen::VectorXd(const Eigen::VectorXd&)>
Eigen::MatrixXd fdJacobianCentral(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
    const Eigen::VectorXd& q,
    double h = 1e-6)
{
    Eigen::VectorXd f0 = f(q);
    int m = static_cast<int>(f0.size());
    int n = static_cast<int>(q.size());
    Eigen::MatrixXd J(m, n);

    for (int j = 0; j < n; ++j) {
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
        dq(j) = h;
        Eigen::VectorXd f_plus  = f(q + dq);
        Eigen::VectorXd f_minus = f(q - dq);
        J.col(j) = (f_plus - f_minus) / (2.0 * h);
    }
    return J;
}
      
