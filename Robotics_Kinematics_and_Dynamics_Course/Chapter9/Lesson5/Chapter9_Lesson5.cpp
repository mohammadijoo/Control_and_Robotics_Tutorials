#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::MatrixXd;

static const double g_acc = 9.81;

Vector2d planar2RGravityTorque(const Vector2d& q,
                               double l1, double l2,
                               double m1, double m2)
{
    double q1 = q(0);
    double q2 = q(1);

    auto potential = [&](const Vector2d& qv) {
        double q1v = qv(0);
        double q2v = qv(1);
        double y1 = (l1 / 2.0) * std::sin(q1v);
        double y2 = l1 * std::sin(q1v) + (l2 / 2.0) * std::sin(q1v + q2v);
        return m1 * g_acc * y1 + m2 * g_acc * y2;
    };

    double eps = 1e-6;
    Vector2d grad;
    for (int i = 0; i < 2; ++i) {
        Vector2d qp = q;
        Vector2d qm = q;
        qp(i) += eps;
        qm(i) -= eps;
        double Vp = potential(qp);
        double Vm = potential(qm);
        grad(i) = (Vp - Vm) / (2.0 * eps);
    }
    return grad;
}

Matrix2d planar2RJacobian(const Vector2d& q,
                          double l1, double l2)
{
    double q1 = q(0);
    double q2 = q(1);
    double s1 = std::sin(q1);
    double c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Matrix2d J;
    J(0,0) = -l1 * s1 - l2 * s12;
    J(0,1) = -l2 * s12;
    J(1,0) =  l1 * c1 + l2 * c12;
    J(1,1) =  l2 * c12;
    return J;
}

Vector2d constrainedStaticTorque(const Vector2d& q,
                                 double l1, double l2,
                                 double m1, double m2,
                                 const VectorXd& wrench6,
                                 double lambda)
{
    Vector2d g_tau = planar2RGravityTorque(q, l1, l2, m1, m2);
    Matrix2d Jxy = planar2RJacobian(q, l1, l2);

    MatrixXd J(6, 2);
    J.setZero();
    J.block<2,2>(0,0) = Jxy;

    Vector2d tau_ext = J.transpose() * wrench6;

    Eigen::RowVector2d Jphi;
    double q1 = q(0);
    double q2 = q(1);
    Jphi(0) = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    Jphi(1) = -l2 * std::sin(q1 + q2);

    Vector2d tau_con = Jphi.transpose() * lambda;

    return g_tau + tau_ext + tau_con;
}

int main()
{
    double l1 = 1.0;
    double l2 = 0.7;
    double m1 = 2.0;
    double m2 = 1.0;

    Vector2d q;
    q(0) = 40.0 * M_PI / 180.0;
    q(1) = 30.0 * M_PI / 180.0;

    VectorXd wrench6(6);
    wrench6.setZero();
    wrench6(1) = -20.0; // Fy

    double lambda = 50.0; // normal reaction at wall
    Vector2d tau = constrainedStaticTorque(q, l1, l2, m1, m2, wrench6, lambda);
    std::cout << "Joint torques: " << tau.transpose() << std::endl;
    return 0;
}
      
