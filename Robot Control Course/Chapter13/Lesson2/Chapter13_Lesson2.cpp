
#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::VectorXd;

Vector2d forwardKinematics2D(const Vector2d& q, double l1, double l2) {
    double q1 = q(0);
    double q2 = q(1);
    double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
    return Vector2d(x, y);
}

VectorXd jointLimitBarrier(const Vector2d& q,
                           const Vector2d& q_min,
                           const Vector2d& q_max,
                           double margin = 0.0) {
    Vector2d h_low = q - (q_min + Vector2d::Constant(margin));
    Vector2d h_up  = (q_max - Vector2d::Constant(margin)) - q;
    VectorXd h(4);
    h.segment<2>(0) = h_low;
    h.segment<2>(2) = h_up;
    return h;
}

double obstacleBarrier(const Vector2d& q,
                       double l1, double l2,
                       const Vector2d& c,
                       double r_obs, double d_min) {
    Vector2d p = forwardKinematics2D(q, l1, l2);
    Vector2d diff = p - c;
    double dist2 = diff.dot(diff);
    double rho = r_obs + d_min;
    return dist2 - rho * rho;
}

bool isSafe(const Vector2d& q,
            const Vector2d& q_min,
            const Vector2d& q_max,
            double l1, double l2,
            const Vector2d& c,
            double r_obs, double d_min,
            double margin = 0.0,
            double tol = 0.0) {
    VectorXd h_joints = jointLimitBarrier(q, q_min, q_max, margin);
    double h_obs = obstacleBarrier(q, l1, l2, c, r_obs, d_min);
    for (int i = 0; i < h_joints.size(); ++i) {
        if (h_joints(i) < -tol) {
            return false;
        }
    }
    if (h_obs < -tol) {
        return false;
    }
    return true;
}

int main() {
    Vector2d q(0.0, 0.0);
    Vector2d q_min(-1.0, -1.0);
    Vector2d q_max( 1.0,  1.0);
    double l1 = 0.8;
    double l2 = 0.6;
    Vector2d c(0.8, 0.0);
    double r_obs = 0.1;
    double d_min = 0.05;

    VectorXd h_joints = jointLimitBarrier(q, q_min, q_max);
    double h_obs = obstacleBarrier(q, l1, l2, c, r_obs, d_min);

    std::cout << "h_joints = " << h_joints.transpose() << std::endl;
    std::cout << "h_obs = " << h_obs << std::endl;
    std::cout << "Is safe? " << isSafe(q, q_min, q_max, l1, l2, c,
                                            r_obs, d_min) << std::endl;
    return 0;
}
