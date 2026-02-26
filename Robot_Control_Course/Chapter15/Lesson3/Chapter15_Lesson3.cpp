
#include <Eigen/Dense>
#include <vector>

class SimpleGP1D {
public:
    using Vec = Eigen::VectorXd;
    using Mat = Eigen::MatrixXd;

    SimpleGP1D(double length_scale,
               double sigma_f,
               double sigma_n)
        : ell_(length_scale),
          sigma_f_(sigma_f),
          sigma_n_(sigma_n),
          trained_(false) {}

    void setTrainingData(const Mat& X, const Vec& y) {
        X_ = X;
        y_ = y;
        const int N = static_cast<int>(y_.size());
        Mat K(N, N);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                K(i, j) = kernel(X_.row(i), X_.row(j));
            }
        }
        Mat Ky = K
               + (sigma_n_ * sigma_n_) * Mat::Identity(N, N);

        // Precompute Ky^{-1} y using LDLT
        Eigen::LDLT<Mat> ldlt(Ky);
        alpha_ = ldlt.solve(y_);
        trained_ = true;
    }

    double predict(const Vec& x_star, double* var_out = nullptr) const {
        assert(trained_);
        const int N = static_cast<int>(y_.size());
        Vec k_star(N);
        for (int i = 0; i < N; ++i) {
            k_star(i) = kernel(X_.row(i), x_star.transpose());
        }
        double mean = k_star.dot(alpha_);

        if (var_out) {
            // For simplicity, we omit efficient reuse of the factorization and
            // compute Ky^{-1} k_star directly.
            const int Nloc = N;
            Mat K(Nloc, Nloc);
            for (int i = 0; i < Nloc; ++i) {
                for (int j = 0; j < Nloc; ++j) {
                    K(i, j) = kernel(X_.row(i), X_.row(j));
                }
            }
            Mat Ky = K
                   + (sigma_n_ * sigma_n_) * Mat::Identity(Nloc, Nloc);
            Eigen::LDLT<Mat> ldlt(Ky);
            Vec v = ldlt.solve(k_star);
            double k_ss = kernel(x_star.transpose(), x_star.transpose());
            *var_out = k_ss - k_star.dot(v);
        }

        return mean;
    }

private:
    double kernel(const Eigen::RowVectorXd& x1,
                  const Eigen::RowVectorXd& x2) const {
        // Isotropic RBF kernel
        double r2 = (x1 - x2).squaredNorm();
        return sigma_f_ * sigma_f_ * std::exp(-0.5 * r2 / (ell_ * ell_));
    }

    Mat X_;
    Vec y_;
    Vec alpha_;      // Ky^{-1} y
    double ell_;
    double sigma_f_;
    double sigma_n_;
    bool trained_;
};

// Example use inside a joint controller loop
double computeTorqueWithGP(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& dqd,
    const Eigen::VectorXd& ddqd,
    SimpleGP1D& gp,
    double tau_nom)  // scalar nominal torque for 1-DOF
{
    Eigen::VectorXd x(3);
    x(0) = q(0);
    x(1) = dq(0);
    x(2) = ddqd(0);
    double d_hat = gp.predict(x);
    double tau = tau_nom + d_hat;
    return tau;
}
