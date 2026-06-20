#include <Eigen/Dense>
#include <cmath>

Eigen::Matrix3d eulerZYXToR(double phi, double theta, double psi) {
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cth  = std::cos(theta), sth = std::sin(theta);
    double cpsi = std::cos(psi), spsi = std::sin(psi);

    Eigen::Matrix3d R;
    R <<
        cpsi * cth,  cpsi * sth * sphi - spsi * cphi,  cpsi * sth * cphi + spsi * sphi,
        spsi * cth,  spsi * sth * sphi + cpsi * cphi,  spsi * sth * cphi - cpsi * sphi,
        -sth,        cth * sphi,                      cth * cphi;
    return R;
}

Eigen::Vector3d eulerZYXToOmega(double phi, double theta,
                                double phi_dot, double theta_dot, double psi_dot) {
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cth  = std::cos(theta), sth = std::sin(theta);

    Eigen::Matrix3d T;
    T <<
        1.0, 0.0, -sth,
        0.0, cphi, cth * sphi,
        0.0, -sphi, cth * cphi;

    Eigen::Vector3d qdot(phi_dot, theta_dot, psi_dot);
    return T * qdot;
}
      
