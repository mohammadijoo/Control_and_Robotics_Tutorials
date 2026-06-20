#include <iostream>
#include <cmath>
#include <Eigen/Dense>

Eigen::Matrix3d RotX(double theta) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,
          0.0,  c,  -s,
          0.0,  s,   c;
    return R;
}

Eigen::Matrix3d RotY(double theta) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d R;
    R <<  c, 0.0,  s,
          0.0, 1.0, 0.0,
          -s, 0.0,  c;
    return R;
}

Eigen::Matrix3d RotZ(double theta) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d R;
    R <<  c, -s, 0.0,
           s,  c, 0.0,
          0.0, 0.0, 1.0;
    return R;
}

bool isSO3(const Eigen::Matrix3d& R, double tol = 1e-9) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d shouldBeI = R.transpose() * R;
    double orthErr = (shouldBeI - I).norm();
    double detR = R.determinant();
    return orthErr < tol && std::abs(detR - 1.0) < tol;
}

int main() {
    double theta = M_PI / 6.0; // 30 degrees

    Eigen::Matrix3d Rz = RotZ(theta);
    Eigen::Matrix3d Ry = RotY(theta);
    Eigen::Matrix3d R = Rz * Ry;

    std::cout << "R =\n" << R << std::endl;
    std::cout << "R^T R =\n" << R.transpose() * R << std::endl;
    std::cout << "det(R) = " << R.determinant() << std::endl;
    std::cout << "R in SO(3)? " << std::boolalpha << isSO3(R) << std::endl;

    // Example: transform coordinates of a vector
    Eigen::Vector3d p_B(1.0, 0.0, 0.0);
    Eigen::Vector3d p_A = R * p_B;
    std::cout << "p_A =\n" << p_A << std::endl;

    return 0;
}
      
