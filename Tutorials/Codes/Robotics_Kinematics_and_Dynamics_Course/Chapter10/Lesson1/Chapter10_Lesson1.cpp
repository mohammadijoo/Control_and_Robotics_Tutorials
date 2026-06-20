#include <iostream>
#include "Eigen/Dense"
#include <cmath>

struct TwoLinkParams {
    double m1, m2;
    double l1, l2;
    double c1, c2;
    double I1, I2;
    double g;
};

Eigen::Matrix2d inertiaMatrix(const Eigen::Vector2d& q,
                              const TwoLinkParams& p)
{
    double q2 = q(1);
    double cos2 = std::cos(q2);

    double M11 = p.I1 + p.I2
               + p.m1 * p.c1 * p.c1
               + p.m2 * (p.l1 * p.l1 + p.c2 * p.c2
                         + 2.0 * p.l1 * p.c2 * cos2);
    double M12 = p.I2 + p.m2 * (p.c2 * p.c2 + p.l1 * p.c2 * cos2);
    double M22 = p.I2 + p.m2 * p.c2 * p.c2;

    Eigen::Matrix2d M;
    M(0,0) = M11; M(0,1) = M12;
    M(1,0) = M12; M(1,1) = M22;
    return M;
}

double potentialEnergy(const Eigen::Vector2d& q,
                       const TwoLinkParams& p)
{
    double q1 = q(0);
    double q2 = q(1);

    double V = p.m1 * p.g * p.c1 * std::sin(q1)
             + p.m2 * p.g * (p.l1 * std::sin(q1)
                             + p.c2 * std::sin(q1 + q2));
    return V;
}

double kineticEnergy(const Eigen::Vector2d& q,
                     const Eigen::Vector2d& qdot,
                     const TwoLinkParams& p)
{
    Eigen::Matrix2d M = inertiaMatrix(q, p);
    return 0.5 * qdot.transpose() * M * qdot;
}

int main()
{
    TwoLinkParams p{2.0, 1.5, 0.4, 0.3, 0.2, 0.15, 0.02, 0.01, 9.81};

    Eigen::Vector2d q(0.5, -0.3);
    Eigen::Vector2d qdot(0.4, 0.2);

    double T = kineticEnergy(q, qdot, p);
    double V = potentialEnergy(q, p);
    double E = T + V;

    std::cout << "T = " << T
              << ", V = " << V
              << ", E = " << E << std::endl;

    return 0;
}
      
