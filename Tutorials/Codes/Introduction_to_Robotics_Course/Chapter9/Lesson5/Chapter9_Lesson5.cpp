#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

Matrix4d makeT(const Matrix3d& R, const Vector3d& p) {
    Matrix4d T = Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = p;
    return T;
}

Matrix4d invT(const Matrix4d& T) {
    Matrix3d R = T.block<3,3>(0,0);
    Vector3d p = T.block<3,1>(0,3);
    Matrix4d Tinv = Matrix4d::Identity();
    Tinv.block<3,3>(0,0) = R.transpose();
    Tinv.block<3,1>(0,3) = -R.transpose() * p;
    return Tinv;
}

double frobError(const Matrix4d& E) {
    return (E - Matrix4d::Identity()).norm(); // Frobenius
}

int main() {
    Matrix3d R_WB = Matrix3d::Identity();
    Vector3d p_WB(1,0,0);
    Matrix4d T_WB = makeT(R_WB, p_WB);

    Matrix3d R_BC = Matrix3d::Identity();
    Vector3d p_BC(0,2,0);
    Matrix4d T_BC = makeT(R_BC, p_BC);

    Matrix4d T_WC_derived = T_WB * T_BC;
    Matrix4d T_WC_meas = T_WC_derived;
    T_WC_meas(0,3) += 0.05;

    Matrix4d E = T_WB * T_BC * invT(T_WC_meas);

    std::cout << "Cycle Frobenius error: " << frobError(E) << std::endl;
    return 0;
}
