#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct CircleSystem {
    double m;
    double l;
    double g;

    CircleSystem(double mass = 1.0, double radius = 1.0, double gravity = 9.81)
        : m(mass), l(radius), g(gravity) {}

    Matrix2d M(const Vector2d& q) const {
        (void)q;
        Matrix2d Mq = Matrix2d::Identity();
        return m * Mq;
    }

    // J_phi(q) = [2x 2y]
    Eigen::RowVector2d J(const Vector2d& q) const {
        return Eigen::RowVector2d(2.0 * q(0), 2.0 * q(1));
    }

    double Jdot_qdot(const Vector2d& q, const Vector2d& qdot) const {
        (void)q;
        double xdot = qdot(0);
        double ydot = qdot(1);
        return 2.0 * (xdot * xdot + ydot * ydot);
    }

    Vector2d unconstrained_forces(const Vector2d& q,
                                  const Vector2d& qdot) const {
        (void)q;
        (void)qdot;
        Vector2d f;
        f << 0.0, -m * g;
        return f;
    }

    // Solve for qddot and lambda
    void step(const Vector2d& q,
              const Vector2d& qdot,
              const Vector2d& tau,
              Vector2d& qddot,
              double& lambda) const
    {
        Matrix2d Mq = M(q);
        Eigen::RowVector2d Jq = J(q);
        double gamma = -Jdot_qdot(q, qdot);
        Vector2d f = unconstrained_forces(q, qdot);

        MatrixXd A(3, 3);
        A.setZero();
        A.block<2,2>(0, 0) = Mq;
        A.block<2,1>(0, 2) = -Jq.transpose();
        A.block<1,2>(2, 0) = Jq;

        VectorXd rhs(3);
        rhs.segment<2>(0) = tau + f;
        rhs(2) = gamma;

        VectorXd sol = A.colPivHouseholderQr().solve(rhs);

        qddot = sol.segment<2>(0);
        lambda = sol(2);
    }
};

int main() {
    CircleSystem sys;
    Vector2d q(1.0, 0.0);
    Vector2d qdot(0.0, 1.0);
    Vector2d tau(0.0, 0.0);
    Vector2d qddot;
    double lambda = 0.0;

    sys.step(q, qdot, tau, qddot, lambda);

    std::cout << "qddot = " << qddot.transpose() << std::endl;
    std::cout << "lambda = " << lambda << std::endl;
    return 0;
}
      
