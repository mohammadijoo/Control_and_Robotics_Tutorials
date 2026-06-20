#include <Eigen/Dense>
#include <cmath>

struct Planar2RParams {
    double m1, m2;
    double l1, lc1, lc2;
    double I1, I2;
    double g0;
};

void planar2R_dynamics(const Eigen::Vector2d& q,
                       const Eigen::Vector2d& qdot,
                       const Planar2RParams& p,
                       Eigen::Matrix2d& M,
                       Eigen::Vector2d& Cqdot,
                       Eigen::Vector2d& g)
{
    const double q1  = q(0);
    const double q2  = q(1);
    const double dq1 = qdot(0);
    const double dq2 = qdot(1);

    const double c2 = std::cos(q2);
    const double s2 = std::sin(q2);

    const double m1  = p.m1;
    const double m2  = p.m2;
    const double l1  = p.l1;
    const double lc1 = p.lc1;
    const double lc2 = p.lc2;
    const double I1  = p.I1;
    const double I2  = p.I2;
    const double g0  = p.g0;

    // Inertia matrix M(q)
    const double M11 = I1 + I2 + m1 * lc1 * lc1
                       + m2 * (l1 * l1 + lc2 * lc2 + 2.0 * l1 * lc2 * c2);
    const double M12 = I2 + m2 * (lc2 * lc2 + l1 * lc2 * c2);
    const double M22 = I2 + m2 * lc2 * lc2;

    M(0,0) = M11;
    M(0,1) = M12;
    M(1,0) = M12;
    M(1,1) = M22;

    // Coriolis/centrifugal term C(q, qdot) qdot
    const double h1 = -m2 * l1 * lc2 * s2 * (2.0 * dq1 * dq2 + dq2 * dq2);
    const double h2 =  m2 * l1 * lc2 * s2 * dq1 * dq1;

    Cqdot(0) = h1;
    Cqdot(1) = h2;

    // Gravity term g(q)
    const double g1 = (m1 * lc1 + m2 * l1) * g0 * std::cos(q1)
                      + m2 * lc2 * g0 * std::cos(q1 + q2);
    const double g2 = m2 * lc2 * g0 * std::cos(q1 + q2);

    g(0) = g1;
    g(1) = g2;
}
      
