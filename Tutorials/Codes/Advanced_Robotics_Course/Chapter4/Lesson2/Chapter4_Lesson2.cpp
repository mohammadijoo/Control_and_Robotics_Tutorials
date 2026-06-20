#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd buildK(int N) {
    MatrixXd K = MatrixXd::Zero(N, N);
    for (int k = 0; k < N; ++k) {
        K(k, k) += 2.0;
        if (k > 0) {
            K(k, k-1) -= 1.0;
            K(k-1, k) -= 1.0;
        }
    }
    return K;
}

double obstacleCost(const Eigen::Vector2d& x) {
    double d0 = 0.5;
    double d = x.norm();
    if (d < d0) {
        double diff = d0 - d;
        return 0.5 * diff * diff;
    }
    return 0.0;
}

Eigen::Vector2d obstacleGrad(const Eigen::Vector2d& x) {
    double d0 = 0.5;
    double d = x.norm();
    if (d < 1e-8) {
        return Eigen::Vector2d::Zero();
    }
    if (d < d0) {
        double coeff = -(d0 - d) / d;
        return coeff * x;
    }
    return Eigen::Vector2d::Zero();
}

void chompStep(std::vector<Eigen::Vector2d>& q,
               const MatrixXd& K, double alpha, double lambda) {
    int N = static_cast<int>(q.size());
    VectorXd q_flat(2 * N);
    for (int k = 0; k < N; ++k) {
        q_flat(2*k)     = q;
        q_flat(2*k + 1) = q;
    }

    MatrixXd Kbig = Eigen::kroneckerProduct(K, MatrixXd::Identity(2, 2)).eval();
    VectorXd grad_smooth = Kbig * q_flat;

    VectorXd grad_obs = VectorXd::Zero(2 * N);
    for (int k = 0; k < N; ++k) {
        Eigen::Vector2d g = obstacleGrad(q[k]);
        grad_obs(2*k)     = g(0);
        grad_obs(2*k + 1) = g(1);
    }

    VectorXd grad_total = grad_smooth + lambda * grad_obs;

    // Solve (Kbig + eps I) delta = grad_total
    MatrixXd A = Kbig + 1e-6 * MatrixXd::Identity(2 * N, 2 * N);
    VectorXd delta = A.ldlt().solve(grad_total);

    VectorXd q_new_flat = q_flat - alpha * delta;
    for (int k = 0; k < N; ++k) {
        q = q_new_flat(2*k);
        q = q_new_flat(2*k + 1);
    }
}
      
