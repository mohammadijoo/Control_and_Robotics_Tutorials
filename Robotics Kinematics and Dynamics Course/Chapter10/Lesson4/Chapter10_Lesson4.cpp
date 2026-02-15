#include <Eigen/Dense>

struct Planar2R {
    double l1, l2, lc1, lc2;
    double m1, m2, I1, I2;
    double g;

    Planar2R(double l1_, double l2_, double lc1_, double lc2_,
             double m1_, double m2_, double I1_, double I2_, double g_ = 9.81)
        : l1(l1_), l2(l2_), lc1(lc1_), lc2(lc2_),
          m1(m1_), m2(m2_), I1(I1_), I2(I2_), g(g_) {}

    Eigen::Matrix2d M(const Eigen::Vector2d& q) const {
        double q2 = q(1);
        double c2 = std::cos(q2);

        double m11 = I1 + I2
                     + m1 * lc1 * lc1
                     + m2 * (l1 * l1 + lc2 * lc2 + 2.0 * l1 * lc2 * c2);
        double m12 = I2 + m2 * (lc2 * lc2 + l1 * lc2 * c2);
        double m22 = I2 + m2 * lc2 * lc2;

        Eigen::Matrix2d Mq;
        Mq(0,0) = m11; Mq(0,1) = m12;
        Mq(1,0) = m12; Mq(1,1) = m22;
        return Mq;
    }

    Eigen::Matrix2d C(const Eigen::Vector2d& q,
                      const Eigen::Vector2d& qd) const {
        double q2 = q(1);
        double q1d = qd(0);
        double q2d = qd(1);
        double s2 = std::sin(q2);
        double h = -m2 * l1 * lc2 * s2;

        Eigen::Matrix2d Cq;
        Cq(0,0) = h * q2d;
        Cq(0,1) = h * (q1d + q2d);
        Cq(1,0) = -h * q1d;
        Cq(1,1) = 0.0;
        return Cq;
    }

    Eigen::Vector2d g_vec(const Eigen::Vector2d& q) const {
        double q1 = q(0);
        double q2 = q(1);
        double g1 = ((m1 * lc1 + m2 * l1) * g * std::cos(q1)
                     + m2 * lc2 * g * std::cos(q1 + q2));
        double g2 = m2 * lc2 * g * std::cos(q1 + q2);
        return Eigen::Vector2d(g1, g2);
    }

    Eigen::Vector2d dynamics(const Eigen::Vector2d& q,
                             const Eigen::Vector2d& qd,
                             const Eigen::Vector2d& u) const {
        Eigen::Matrix2d Mq = M(q);
        Eigen::Matrix2d Cq = C(q, qd);
        Eigen::Vector2d gq = g_vec(q);
        Eigen::Vector2d rhs = u - Cq * qd - gq;
        return Mq.ldlt().solve(rhs); // qdd
    }
};
      
