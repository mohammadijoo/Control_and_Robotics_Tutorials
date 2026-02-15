#include <Eigen/Dense>
#include <iostream>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

struct ConstrainedMassOnCircle {
    double m{1.0};
    double g{9.81};
    double R{1.0};
    double alpha{10.0};
    double beta{20.0};

    double phi(const Vector2d& q) const {
        double x = q(0), y = q(1);
        return x * x + y * y - R * R;
    }

    Eigen::RowVector2d J(const Vector2d& q) const {
        double x = q(0), y = q(1);
        Eigen::RowVector2d Jq;
        Jq << 2.0 * x, 2.0 * y;
        return Jq;
    }

    Eigen::RowVector2d Jdot(const Vector2d& v) const {
        double vx = v(0), vy = v(1);
        Eigen::RowVector2d Jd;
        Jd << 2.0 * vx, 2.0 * vy;
        return Jd;
    }

    void step(Vector2d& q, Vector2d& v, double h) const {
        Matrix2d M = m * Matrix2d::Identity();
        Vector2d g_vec(0.0, m * g);

        Eigen::RowVector2d Jq = J(q);
        Eigen::RowVector2d Jd = Jdot(v);

        double phi_q = phi(q);
        double Jv = Jq * v;
        double Jdv = Jd * v;

        Matrix3d A = Matrix3d::Zero();
        // M block
        A.block<2,2>(0,0) = M;
        // J^T block
        A.block<2,1>(0,2) = Jq.transpose();
        // J block
        A.block<1,2>(2,0) = Jq;

        Vector3d rhs = Vector3d::Zero();
        rhs.segment<2>(0) = -g_vec;
        rhs(2) = -Jdv - 2.0 * alpha * Jv - beta * beta * phi_q;

        Vector3d sol = A.fullPivLu().solve(rhs);
        Vector2d vdot = sol.segment<2>(0);
        // double lambda = sol(2);

        v += h * vdot;
        q += h * v;
    }
};

int main() {
    ConstrainedMassOnCircle sys;
    double h = 1e-3;
    int n_steps = 10000;

    Vector2d q(sys.R, 0.0);
    Vector2d v(0.0, 0.5);

    for (int k = 0; k < n_steps; ++k) {
        sys.step(q, v, h);
    }
    std::cout << "Final phi(q): " << sys.phi(q) << std::endl;
    return 0;
}
      
