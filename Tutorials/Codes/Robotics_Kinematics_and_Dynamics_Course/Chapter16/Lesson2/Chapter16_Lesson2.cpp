#include <Eigen/Dense>

struct Planar3RPRGeometry {
    Eigen::Vector2d B[3]; // base anchor points
    Eigen::Vector2d P[3]; // platform points (platform frame)
};

inline Eigen::Matrix2d rot2(double phi) {
    double c = std::cos(phi);
    double s = std::sin(phi);
    Eigen::Matrix2d R;
    R << c, -s,
         s,  c;
    return R;
}

inline Eigen::Matrix2d S_matrix() {
    Eigen::Matrix2d S;
    S << 0.0, -1.0,
         1.0,  0.0;
    return S;
}

void computeConstraintJacobians3RPR(
    const Planar3RPRGeometry& geom,
    const Eigen::Vector3d& L,      // [L1, L2, L3]
    const Eigen::Vector3d& pose,   // [x, y, phi]
    Eigen::Matrix3d& Jq,
    Eigen::Matrix3d& Jx,
    Eigen::Matrix3d& Jf)
{
    double x = pose(0);
    double y = pose(1);
    double phi = pose(2);

    Eigen::Vector2d p(x, y);
    Eigen::Matrix2d R = rot2(phi);
    Eigen::Matrix2d S = S_matrix();

    // J_q: diagonal with entries -2 * L_i
    Jq.setZero();
    for (int i = 0; i < 3; ++i) {
        Jq(i, i) = -2.0 * L(i);
    }

    // J_x rows
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector2d d = p + R * geom.P[i] - geom.B[i];
        double dix = d(0);
        double diy = d(1);

        Eigen::Vector2d dphi = S * (R * geom.P[i]);

        double dPhi_dx = 2.0 * dix;
        double dPhi_dy = 2.0 * diy;
        double dPhi_dphi = 2.0 * d.dot(dphi);

        Jx(i, 0) = dPhi_dx;
        Jx(i, 1) = dPhi_dy;
        Jx(i, 2) = dPhi_dphi;
    }

    // J_f = - J_x^{-1} J_q
    Jf = - Jx.inverse() * Jq;
}
      
