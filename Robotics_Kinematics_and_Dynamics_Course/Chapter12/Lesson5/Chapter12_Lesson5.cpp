#include <iostream>
#include <cmath>
#include <Eigen/Dense>

struct TwoLinkParams {
    double m1, m2;
    double l1, c1, c2;
    double I1, I2;
    double g;
};

Eigen::Matrix2d M_lagrange_2R(const Eigen::Vector2d& q,
                              const TwoLinkParams& p) {
    double q2 = q(1);
    double a = p.I1 + p.I2 + p.m1 * p.c1 * p.c1
               + p.m2 * (p.l1 * p.l1 + p.c2 * p.c2);
    double b = p.m2 * p.l1 * p.c2;
    double d = p.I2 + p.m2 * p.c2 * p.c2;
    double c2 = std::cos(q2);

    Eigen::Matrix2d M;
    M(0,0) = a + 2.0 * b * c2;
    M(0,1) = d + b * c2;
    M(1,0) = M(0,1);
    M(1,1) = d;
    return M;
}

Eigen::Vector2d h_lagrange_2R(const Eigen::Vector2d& q,
                              const Eigen::Vector2d& dq,
                              const TwoLinkParams& p) {
    double q2  = q(1);
    double dq1 = dq(0);
    double dq2 = dq(1);
    double b   = p.m2 * p.l1 * p.c2;
    double s2  = std::sin(q2);

    Eigen::Vector2d h;
    h(0) = -b * s2 * (2.0 * dq1 * dq2 + dq2 * dq2);
    h(1) =  b * s2 * dq1 * dq1;
    return h;
}

Eigen::Vector2d g_lagrange_2R(const Eigen::Vector2d& q,
                              const TwoLinkParams& p) {
    double q1 = q(0);
    double q2 = q(1);

    double g1 = p.g * (p.c1 * p.m1 * std::cos(q1)
                       + p.m2 * (p.l1 * std::cos(q1)
                                 + p.c2 * std::cos(q1 + q2)));
    double g2 = p.g * (p.m2 * p.c2 * std::cos(q1 + q2));

    Eigen::Vector2d g;
    g(0) = g1;
    g(1) = g2;
    return g;
}

Eigen::Vector2d tau_lagrange_2R(const Eigen::Vector2d& q,
                                const Eigen::Vector2d& dq,
                                const Eigen::Vector2d& ddq,
                                const TwoLinkParams& p) {
    Eigen::Matrix2d M = M_lagrange_2R(q, p);
    Eigen::Vector2d h = h_lagrange_2R(q, dq, p);
    Eigen::Vector2d g = g_lagrange_2R(q, p);
    return M * ddq + h + g;
}

// ------------------------------------------------------------------
// Newton-Euler function to be supplied by you (Lesson 4).
// g_base is the 3D gravity vector in the base frame.
// ------------------------------------------------------------------
Eigen::Vector2d tau_newton_euler_2R(const Eigen::Vector2d& q,
                                    const Eigen::Vector2d& dq,
                                    const Eigen::Vector2d& ddq,
                                    const TwoLinkParams& p,
                                    const Eigen::Vector3d& g_base) {
    // TODO: call your library / implementation here.
    throw std::runtime_error("Connect to your Newton-Euler implementation.");
}

Eigen::Matrix2d reconstruct_M_NE(const Eigen::Vector2d& q,
                                 const TwoLinkParams& p) {
    Eigen::Matrix2d M;
    Eigen::Vector2d dq = Eigen::Vector2d::Zero();
    Eigen::Vector3d g_base(0.0, 0.0, 0.0); // zero gravity

    for (int j = 0; j < 2; ++j) {
        Eigen::Vector2d ddq = Eigen::Vector2d::Zero();
        ddq(j) = 1.0;
        Eigen::Vector2d tau =
            tau_newton_euler_2R(q, dq, ddq, p, g_base);
        M.col(j) = tau;
    }
    return M;
}

Eigen::Vector2d reconstruct_g_NE(const Eigen::Vector2d& q,
                                 const TwoLinkParams& p) {
    Eigen::Vector2d dq  = Eigen::Vector2d::Zero();
    Eigen::Vector2d ddq = Eigen::Vector2d::Zero();
    Eigen::Vector3d g_base(0.0, -p.g, 0.0);
    return tau_newton_euler_2R(q, dq, ddq, p, g_base);
}

int main() {
    TwoLinkParams p;
    p.m1 = 2.0; p.m2 = 1.0;
    p.l1 = 1.0; p.c1 = 0.5; p.c2 = 0.5;
    p.I1 = 0.2; p.I2 = 0.1;
    p.g  = 9.81;

    Eigen::Vector2d q(0.3, -0.7);
    Eigen::Matrix2d M_L  = M_lagrange_2R(q, p);
    Eigen::Matrix2d M_NE = reconstruct_M_NE(q, p);

    std::cout << "M_L(q) =\n" << M_L << std::endl;
    std::cout << "M_NE(q) =\n" << M_NE << std::endl;
    std::cout << "M_L - M_NE =\n" << (M_L - M_NE) << std::endl;

    Eigen::Vector2d g_L  = g_lagrange_2R(q, p);
    Eigen::Vector2d g_NE = reconstruct_g_NE(q, p);
    std::cout << "g_L(q)  = " << g_L.transpose()  << std::endl;
    std::cout << "g_NE(q) = " << g_NE.transpose() << std::endl;

    return 0;
}
      
