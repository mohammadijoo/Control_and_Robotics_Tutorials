#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

bool isSPD(const MatrixXd& M, double tol = 1e-9) {
    if (M.rows() != M.cols()) return false;
    Eigen::LLT<MatrixXd> llt(M);
    if (llt.info() != Eigen::Success) return false;
    // Optional: check diagonal > tol
    return (llt.matrixL().diagonal().array() > tol).all();
}

Matrix3d skew(const Vector3d& v) {
    Matrix3d S;
    S << 0.0,   -v.z(),  v.y(),
           v.z(),  0.0,   -v.x(),
          -v.y(),  v.x(),  0.0;
    return S;
}

Eigen::Matrix<double,6,6> spatialInertia(double m,
                                          const Vector3d& c,
                                          const Matrix3d& I_C)
{
    Matrix3d S = skew(c);
    Matrix3d upper_left = I_C + m * S * S.transpose();
    Matrix3d upper_right = m * S;
    Matrix3d lower_left = m * S.transpose();
    Matrix3d lower_right = m * Matrix3d::Identity();

    Eigen::Matrix<double,6,6> I;
    I.topLeftCorner<3,3>() = upper_left;
    I.topRightCorner<3,3>() = upper_right;
    I.bottomLeftCorner<3,3>() = lower_left;
    I.bottomRightCorner<3,3>() = lower_right;
    return I;
}

int main() {
    double m = 3.0;
    Vector3d c(0.0, 0.0, 0.1);
    Matrix3d I_C;
    I_C << 0.02, 0.0,  0.0,
            0.0,  0.03, 0.0,
            0.0,  0.0,  0.01;

    if (m <= 0.0) {
        std::cerr << "Non-positive mass!" << std::endl;
        return 1;
    }

    // Symmetrize
    I_C = 0.5 * (I_C + I_C.transpose());

    if (!isSPD(I_C)) {
        std::cerr << "Inertia about COM is not SPD!" << std::endl;
    }

    Eigen::Matrix<double,6,6> I_spatial = spatialInertia(m, c, I_C);
    if (!isSPD(I_spatial)) {
        std::cerr << "Spatial inertia is not SPD!" << std::endl;
    } else {
        std::cout << "Spatial inertia is SPD and physically consistent." << std::endl;
    }

    // In RBDL or Pinocchio, analogous checks can be performed on
    // rbdl::Model::I bodies or pinocchio::Inertia objects.
    return 0;
}
      
