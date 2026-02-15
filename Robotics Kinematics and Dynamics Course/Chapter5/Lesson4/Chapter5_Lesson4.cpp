#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct DHParam {
    double a;
    double alpha;
    double d;
    double theta_offset;
    bool   revolute;
};

Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta) {
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);
    double ct = std::cos(theta);
    double st = std::sin(theta);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,0) = ct;   T(0,1) = -st * ca;  T(0,2) =  st * sa;  T(0,3) = a * ct;
    T(1,0) = st;   T(1,1) =  ct * ca;  T(1,2) = -ct * sa;  T(1,3) = a * st;
    T(2,0) = 0.0;  T(2,1) =  sa;       T(2,2) =  ca;       T(2,3) = d;
    return T;
}

Eigen::Matrix4d fkDH(const std::vector<DHParam>& dh,
                     const std::vector<double>& q) {
    std::size_t n = dh.size();
    if (q.size() != n) {
        throw std::runtime_error("fkDH: size mismatch between DH table and q.");
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0; i < n; ++i) {
        double theta = dh[i].theta_offset;
        double d = dh[i].d;
        if (dh[i].revolute) {
            theta += q[i];
        } else {
            d += q[i];
        }
        Eigen::Matrix4d Ti = dhTransform(dh[i].a, dh[i].alpha, d, theta);
        T = T * Ti;
    }
    return T;
}

int main() {
    // Example: 7-DOF redundant arm approximated by 7 revolute joints
    std::vector<DHParam> dh(7);
    for (int i = 0; i < 7; ++i) {
        dh[i].a = 0.2;
        dh[i].alpha = 0.0;
        dh[i].d = 0.0;
        dh[i].theta_offset = 0.0;
        dh[i].revolute = true;
    }

    std::vector<double> q = {0.0, 0.3, -0.5, 0.4, -0.2, 0.1, 0.05};
    Eigen::Matrix4d T = fkDH(dh, q);
    std::cout << "End-effector transform:\n" << T << std::endl;

    return 0;
}
      
