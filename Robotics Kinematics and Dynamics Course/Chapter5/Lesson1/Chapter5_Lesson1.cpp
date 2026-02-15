#include <iostream>
#include <vector>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::VectorXd;

Matrix3d skew3(const Vector3d& omega) {
    Matrix3d wx;
    wx << 0.0,      -omega(2),  omega(1),
           omega(2),  0.0,       -omega(0),
          -omega(1),  omega(0),  0.0;
    return wx;
}

Matrix3d matrixExp3(const Vector3d& omega, double theta) {
    Matrix3d wx = skew3(omega);
    double theta2 = theta * theta;
    return Matrix3d::Identity()
         + std::sin(theta) * wx
         + (1.0 - std::cos(theta)) * (wx * wx);
}

Matrix4d matrixExp6(const Eigen::Matrix<double, 6, 1>& S, double theta) {
    Vector3d omega = S.segment<3>(0);
    Vector3d v     = S.segment<3>(3);

    double norm_w = omega.norm();
    Matrix3d R;
    Vector3d p;

    if (norm_w > 1e-8) {
        Vector3d omega_unit = omega / norm_w;
        double theta_scaled = norm_w * theta;
        R = matrixExp3(omega_unit, theta_scaled);

        Matrix3d wx = skew3(omega_unit);
        Matrix3d G = Matrix3d::Identity() * theta_scaled
                   + (1.0 - std::cos(theta_scaled)) * wx
                   + (theta_scaled - std::sin(theta_scaled)) * (wx * wx);
        p = G * (v / norm_w);
    } else {
        R = Matrix3d::Identity();
        p = v * theta;
    }

    Matrix4d T = Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = p;
    return T;
}

Matrix4d fkine_space(const Matrix4d& M,
                     const std::vector<Eigen::Matrix<double, 6, 1>>& S_list,
                     const VectorXd& theta) {
    Matrix4d T = Matrix4d::Identity();
    std::size_t n = S_list.size();
    for (std::size_t i = 0; i < n; ++i) {
        T = T * matrixExp6(S_list[i], theta(static_cast<int>(i)));
    }
    return T * M;
}

int main() {
    // Example: 2R planar arm
    double L1 = 1.0;
    double L2 = 1.0;

    Eigen::Matrix<double, 6, 1> S1;
    Eigen::Matrix<double, 6, 1> S2;
    S1 << 0.0, 0.0, 1.0, 0.0,  0.0, 0.0;
    S2 << 0.0, 0.0, 1.0, 0.0, -L1, 0.0;

    std::vector<Eigen::Matrix<double, 6, 1>> S_list = {S1, S2};

    Matrix4d M = Matrix4d::Identity();
    M(0,3) = L1 + L2;

    VectorXd theta(2);
    theta(0) = 30.0 * M_PI / 180.0;
    theta(1) = 45.0 * M_PI / 180.0;

    Matrix4d T = fkine_space(M, S_list, theta);
    std::cout << "T(theta) =\n" << T << std::endl;
    return 0;
}
      
