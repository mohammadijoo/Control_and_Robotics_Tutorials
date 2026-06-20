#include <iostream>
#include <cmath>
#include <Eigen/Dense>

Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Eigen::Matrix4d T;
    T <<
        ct, -st * ca,  st * sa, a * ct,
        st,  ct * ca, -ct * sa, a * st,
        0.0,     sa,      ca,      d,
        0.0,    0.0,     0.0,    1.0;
    return T;
}

Eigen::Matrix4d forwardKinematicsDH(
    const std::vector<std::array<double,4>> &dhTable)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (const auto &row : dhTable) {
        double theta = row[0];
        double d     = row[1];
        double a     = row[2];
        double alpha = row[3];
        T = T * dhTransform(theta, d, a, alpha);
    }
    return T;
}

int main() {
    double l1 = 1.0, l2 = 0.7;
    double q1 = 0.5, q2 = -0.3;

    std::vector<std::array<double,4>> dh = {
        { q1, 0.0, l1, 0.0 },
        { q2, 0.0, l2, 0.0 }
    };

    Eigen::Matrix4d T = forwardKinematicsDH(dh);
    std::cout << "T_0_2 =\n" << T << std::endl;
    std::cout << "End-effector position: "
              << T(0,3) << ", " << T(1,3) << ", " << T(2,3)
              << std::endl;
    return 0;
}
      
