#include <Eigen/Dense>
#include <vector>
#include <cstddef>

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

Vec2 planar2rFK(const Vec2& q, double l1, double l2) {
    double q1 = q(0);
    double q2 = q(1);
    double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
    return Vec2(x, y);
}

Mat2 planar2rJ(const Vec2& q, double l1, double l2) {
    double q1 = q(0);
    double q2 = q(1);
    double s1 = std::sin(q1);
    double c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Mat2 J;
    J(0, 0) = -l1 * s1 - l2 * s12;
    J(0, 1) = -l2 * s12;
    J(1, 0) =  l1 * c1 + l2 * c12;
    J(1, 1) =  l2 * c12;
    return J;
}

void centralDiff(const std::vector<Vec2>& x, double h, std::vector<Vec2>& dx) {
    std::size_t N = x.size();
    dx.resize(N);
    if (N < 2) {
        return;
    }
    dx[0].setZero();
    dx[N - 1].setZero();
    for (std::size_t k = 1; k + 1 < N; ++k) {
        dx[k] = (x[k + 1] - x[k - 1]) / (2.0 * h);
    }
}

int main() {
    double l1 = 1.0;
    double l2 = 0.7;
    double omega = 1.5;
    double T = 5.0;
    std::size_t N = 501;
    double h = T / (static_cast<double>(N - 1));

    std::vector<Vec2> q(N), dq_ana(N), x(N), xd_num(N), xd_model(N);

    for (std::size_t k = 0; k < N; ++k) {
        double t = static_cast<double>(k) * h;
        q = std::sin(omega * t);
        q = std::cos(omega * t);

        dq_ana = omega * std::cos(omega * t);
        dq_ana = -omega * std::sin(omega * t);

        x[k] = planar2rFK(q[k], l1, l2);
    }

    // numerical differentiation of x
    centralDiff(x, h, xd_num);

    // model-based xdot
    for (std::size_t k = 0; k < N; ++k) {
        Mat2 J = planar2rJ(q[k], l1, l2);
        xd_model[k] = J * dq_ana[k];
    }

    // (RMS error computation omitted for brevity)
    return 0;
}
      
