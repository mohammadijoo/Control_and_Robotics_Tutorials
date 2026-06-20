#include <Eigen/Dense>
#include <iostream>

struct RigidBodyConfig {
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
};

bool isValidRotation(const Eigen::Matrix3d& R, double tol = 1e-6) {
    Eigen::Matrix3d RT_R = R.transpose() * R;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    double orth_error = (RT_R - I).norm();
    double det_error = std::abs(R.determinant() - 1.0);
    return (orth_error <= tol) && (det_error <= tol);
}

int main() {
    RigidBodyConfig q;
    q.R = Eigen::Matrix3d::Identity();
    q.p = Eigen::Vector3d::Zero();

    std::cout << "Valid configuration: "
              << (isValidRotation(q.R) ? "yes" : "no")
              << std::endl;

    // Simple joint-space example: planar 2R arm
    Eigen::Vector2d q_joint;
    q_joint << 0.5, -0.3;
    std::cout << "q = [" << q_joint(0) << ", " << q_joint(1) << "]" << std::endl;
    return 0;
}
      
