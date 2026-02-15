#include <iostream>
#include <Eigen/Dense>

Eigen::MatrixXd finiteDifferenceJerk(const Eigen::MatrixXd& q_samples, double dt) {
    // q_samples: (N+1) x n_joints
    const std::ptrdiff_t N_plus_1 = q_samples.rows();
    const std::ptrdiff_t n_joints = q_samples.cols();

    if (N_plus_1 < 4) {
        throw std::runtime_error("Need at least 4 samples to compute third derivative");
    }

    const std::ptrdiff_t N = N_plus_1 - 1;
    Eigen::MatrixXd jerk(N - 2, n_joints);
    double dt3 = dt * dt * dt;

    for (std::ptrdiff_t k = 1; k < N - 1; ++k) {
        jerk.row(k - 1) =
            (q_samples.row(k + 2)
            - 3.0 * q_samples.row(k + 1)
            + 3.0 * q_samples.row(k)
            - q_samples.row(k - 1)) / dt3;
    }
    return jerk;
}

double jerkSmoothnessIndex(const Eigen::MatrixXd& q_samples, double dt,
                           const Eigen::MatrixXd* W_ptr = nullptr) {
    Eigen::MatrixXd jerk = finiteDifferenceJerk(q_samples, dt);
    const std::ptrdiff_t n_joints = jerk.cols();

    Eigen::MatrixXd W;
    if (W_ptr == nullptr) {
        W = Eigen::MatrixXd::Identity(n_joints, n_joints);
    } else {
        W = *W_ptr;
        if (W.rows() != n_joints || W.cols() != n_joints) {
            throw std::runtime_error("Weight matrix W has wrong dimensions");
        }
    }

    double J = 0.0;
    for (std::ptrdiff_t k = 0; k < jerk.rows(); ++k) {
        Eigen::VectorXd j_k = jerk.row(k).transpose();
        J += j_k.transpose() * W * j_k;
    }
    J *= dt;
    return J;
}

int main() {
    // Example: single joint, smooth scalar trajectory, as in the Python example.
    double T = 2.0;
    double dt = 0.01;
    int N_plus_1 = static_cast<int>(T / dt) + 1;
    Eigen::MatrixXd q_samples(N_plus_1, 1);

    for (int k = 0; k < N_plus_1; ++k) {
        double t = k * dt;
        double s = t / T;
        double q = 10.0 * s * s * s
                 - 15.0 * s * s * s * s
                 + 6.0 * s * s * s * s * s;
        q_samples(k, 0) = q;
    }

    Eigen::MatrixXd jerk = finiteDifferenceJerk(q_samples, dt);
    double J = jerkSmoothnessIndex(q_samples, dt);

    std::cout << "Jerk samples: " << jerk.rows() << " x "
              << jerk.cols() << std::endl;
    std::cout << "Smoothness index J = " << J << std::endl;
    return 0;
}
      
