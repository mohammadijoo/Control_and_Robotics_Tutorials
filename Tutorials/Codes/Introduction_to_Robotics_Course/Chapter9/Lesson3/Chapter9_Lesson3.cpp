#include <iostream>
#include <Eigen/Dense>

Eigen::Matrix3d Rz(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

Eigen::Vector3d rigidApply(const Eigen::Matrix3d& R,
                           const Eigen::Vector3d& p,
                           const Eigen::Vector3d& x) {
    return R * x + p;
}

void rigidCompose(const Eigen::Matrix3d& R_cb, const Eigen::Vector3d& p_cb,
                  const Eigen::Matrix3d& R_ba, const Eigen::Vector3d& p_ba,
                  Eigen::Matrix3d& R_ca, Eigen::Vector3d& p_ca) {
    R_ca = R_cb * R_ba;
    p_ca = R_cb * p_ba + p_cb;
}

int main() {
    double theta = M_PI / 4.0;
    Eigen::Matrix3d R = Rz(theta);
    Eigen::Vector3d p(0.5, -0.2, 0.1);
    Eigen::Vector3d xA(1.0, 0.0, 0.0);

    Eigen::Vector3d xB = rigidApply(R, p, xA);
    std::cout << "xB = " << xB.transpose() << std::endl;
    return 0;
}
