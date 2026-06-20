#include <iostream>
#include <Eigen/Dense>

struct RigidBodyInertia {
    double mass;
    Eigen::Vector3d com;     // CoM in link frame
    Eigen::Matrix3d I_origin; // Inertia about link origin
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0.0,   -v.z(),  v.y(),
           v.z(),  0.0,   -v.x(),
          -v.y(),  v.x(),  0.0;
    return S;
}

Eigen::Matrix<double,6,6> spatialInertia(const RigidBodyInertia& rb) {
    const double m = rb.mass;
    const Eigen::Matrix3d S = skew(rb.com);

    Eigen::Matrix<double,6,6> I = Eigen::Matrix<double,6,6>::Zero();
    I.block<3,3>(0,0) = rb.I_origin + m * S * S.transpose();
    I.block<3,3>(0,3) = m * S;
    I.block<3,3>(3,0) = -m * S;
    I.block<3,3>(3,3) = m * Eigen::Matrix3d::Identity();
    return I;
}

int main() {
    RigidBodyInertia link;
    link.mass = 5.0;
    link.com  = Eigen::Vector3d(0.1, 0.0, 0.0);
    link.I_origin <<
        0.2, 0.0, 0.0,
        0.0, 0.3, 0.0,
        0.0, 0.0, 0.4;

    Eigen::Matrix<double,6,6> Isp = spatialInertia(link);

    std::cout << "Spatial inertia:\n" << Isp << std::endl;
    return 0;
}
      
