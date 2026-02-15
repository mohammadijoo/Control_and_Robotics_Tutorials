
#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Matrix2d;

struct Planar2DOF {
    double l1{1.0};
    double l2{1.0};

    Vector2d fk(const Vector2d& q) const {
        double q1 = q(0), q2 = q(1);
        double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
        double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
        return Vector2d(x, y);
    }

    Matrix2d jacobian(const Vector2d& q) const {
        double q1 = q(0), q2 = q(1);
        Matrix2d J;
        J(0,0) = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
        J(0,1) = -l2 * std::sin(q1 + q2);
        J(1,0) =  l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
        J(1,1) =  l2 * std::cos(q1 + q2);
        return J;
    }
};

// Placeholder inverse dynamics (should be replaced by real dynamics)
Vector2d inverseDynamics(const Vector2d& q,
                         const Vector2d& qd,
                         const Vector2d& qdd_des)
{
    return qdd_des; // stand-in for tau
}

Vector2d jointSpacePD(const Vector2d& q,
                      const Vector2d& qd,
                      const Vector2d& q_des,
                      const Vector2d& qd_des,
                      const Matrix2d& Kp,
                      const Matrix2d& Kd)
{
    Vector2d e_q  = q_des - q;
    Vector2d e_qd = qd_des - qd;
    Vector2d qdd_des = Kp * e_q + Kd * e_qd;
    return inverseDynamics(q, qd, qdd_des);
}

Vector2d taskSpacePD(const Planar2DOF& arm,
                     const Vector2d& q,
                     const Vector2d& qd,
                     const Vector2d& x_des,
                     const Vector2d& xd_des,
                     const Matrix2d& Kx,
                     const Matrix2d& Dx)
{
    Vector2d x = arm.fk(q);
    Matrix2d J = arm.jacobian(q);
    Vector2d xdot = J * qd;

    Vector2d e_x  = x_des  - x;
    Vector2d e_xd = xd_des - xdot;

    Vector2d xdd_des = Kx * e_x + Dx * e_xd;

    // Pseudoinverse of 2x2 matrix (non-singular assumption)
    Matrix2d Jinv = J.inverse();
    Vector2d qdd_des = Jinv * xdd_des;

    return inverseDynamics(q, qd, qdd_des);
}

int main() {
    Planar2DOF arm;
    Vector2d q(0.4, 0.2);
    Vector2d qd(0.0, 0.0);
    Vector2d q_des(0.5, 0.3);
    Vector2d qd_des(0.0, 0.0);

    Vector2d x_des = arm.fk(q_des);
    Vector2d xd_des(0.0, 0.0);

    Matrix2d Kp = Matrix2d::Identity() * 30.0;
    Matrix2d Kd = Matrix2d::Identity() * 10.0;
    Matrix2d Kx = Matrix2d::Identity() * 50.0;
    Matrix2d Dx = Matrix2d::Identity() * 20.0;

    Vector2d tau_joint = jointSpacePD(q, qd, q_des, qd_des, Kp, Kd);
    Vector2d tau_task  = taskSpacePD(arm, q, qd, x_des, xd_des, Kx, Dx);

    std::cout << "Tau (joint-space): " << tau_joint.transpose() << std::endl;
    std::cout << "Tau (task-space):  " << tau_task.transpose()  << std::endl;
    return 0;
}
