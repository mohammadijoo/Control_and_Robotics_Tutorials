#include <Eigen/Dense>

struct TwoLinkParams {
    double l1, l2;
    double c1, c2;
    double m1, m2;
    double I1, I2;
    double g;
};

Eigen::Matrix2d M_2R(const Eigen::Vector2d& q, const TwoLinkParams& p) {
    double q2 = q(1);
    double cos2 = std::cos(q2);

    double M11 = p.I1 + p.I2
                 + p.m1 * p.c1 * p.c1
                 + p.m2 * (p.l1 * p.l1 + p.c2 * p.c2 + 2.0 * p.l1 * p.c2 * cos2);
    double M12 = p.I2 + p.m2 * (p.c2 * p.c2 + p.l1 * p.c2 * cos2);
    double M22 = p.I2 + p.m2 * p.c2 * p.c2;

    Eigen::Matrix2d M;
    M(0,0) = M11; M(0,1) = M12;
    M(1,0) = M12; M(1,1) = M22;
    return M;
}

Eigen::Matrix2d C_2R(const Eigen::Vector2d& q,
                     const Eigen::Vector2d& qdot,
                     const TwoLinkParams& p) {
    double q2 = q(1);
    double q1dot = qdot(0);
    double q2dot = qdot(1);

    double sin2 = std::sin(q2);
    double h = p.m2 * p.l1 * p.c2 * sin2;

    Eigen::Matrix2d C;
    C(0,0) = -h * q2dot;
    C(0,1) = -h * (q1dot + q2dot);
    C(1,0) =  h * q1dot;
    C(1,1) =  0.0;
    return C;
}

Eigen::Vector2d g_2R(const Eigen::Vector2d& q, const TwoLinkParams& p) {
    double q1 = q(0);
    double q2 = q(1);

    double g1 = (p.m1 * p.c1 + p.m2 * p.l1) * p.g * std::cos(q1)
                + p.m2 * p.c2 * p.g * std::cos(q1 + q2);
    double g2 = p.m2 * p.c2 * p.g * std::cos(q1 + q2);

    Eigen::Vector2d gv;
    gv(0) = g1;
    gv(1) = g2;
    return gv;
}

Eigen::Vector2d forward_dynamics_2R(const Eigen::Vector2d& q,
                                    const Eigen::Vector2d& qdot,
                                    const Eigen::Vector2d& tau,
                                    const TwoLinkParams& p) {
    Eigen::Matrix2d M = M_2R(q, p);
    Eigen::Matrix2d C = C_2R(q, qdot, p);
    Eigen::Vector2d gvec = g_2R(q, p);

    Eigen::Vector2d rhs = tau - C * qdot - gvec;
    Eigen::Vector2d qddot = M.ldlt().solve(rhs);
    return qddot;
}
      
