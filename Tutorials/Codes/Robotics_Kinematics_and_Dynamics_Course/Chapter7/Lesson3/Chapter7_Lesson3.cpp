#include <iostream>
#include <vector>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

Matrix3d skew(const Vector3d& v) {
    Matrix3d S;
    S <<     0.0, -v(2),  v(1),
           v(2),    0.0, -v(0),
          -v(1),  v(0),   0.0;
    return S;
}

Matrix4d se3Hat(const VectorXd& S) {
    Matrix4d M = Matrix4d::Zero();
    Vector3d omega = S.segment<3>(0);
    Vector3d v = S.segment<3>(3);
    M.block<3,3>(0,0) = skew(omega);
    M.block<3,1>(0,3) = v;
    return M;
}

// Exponential for twist S and scalar theta
Matrix4d se3Exp(const VectorXd& S, double theta, double tol=1e-9) {
    Matrix4d result = Matrix4d::Identity();
    Vector3d omega = S.segment<3>(0);
    Vector3d v = S.segment<3>(3);
    double wnorm = omega.norm();
    if (wnorm < tol) {
        // Pure translation
        result.block<3,1>(0,3) = v * theta;
        return result;
    }
    Vector3d w = omega / wnorm;
    Matrix3d w_hat = skew(w);
    double th = wnorm * theta;
    Matrix3d R = Matrix3d::Identity()
                 + std::sin(th) * w_hat
                 + (1.0 - std::cos(th)) * (w_hat * w_hat);
    Matrix3d G = Matrix3d::Identity() * th
                 + (1.0 - std::cos(th)) * w_hat
                 + (th - std::sin(th)) * (w_hat * w_hat);
    Vector3d p = G * (v / wnorm);
    result.block<3,3>(0,0) = R;
    result.block<3,1>(0,3) = p;
    return result;
}

Eigen::Matrix<double,6,6> adjoint(const Matrix4d& T) {
    Eigen::Matrix<double,6,6> Ad = Eigen::Matrix<double,6,6>::Zero();
    Matrix3d R = T.block<3,3>(0,0);
    Vector3d p = T.block<3,1>(0,3);
    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(3,3) = R;
    Ad.block<3,3>(3,0) = skew(p) * R;
    return Ad;
}

Matrix4d fkPoeSpace(const std::vector<VectorXd>& Slist,
                    const Matrix4d& M,
                    const VectorXd& q) {
    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < (int)Slist.size(); ++i) {
        T = T * se3Exp(Slist[i], q(i));
    }
    T = T * M;
    return T;
}

MatrixXd jacobianSpace(const std::vector<VectorXd>& Slist,
                       const VectorXd& q) {
    int n = (int)Slist.size();
    MatrixXd J(6, n);
    J.setZero();
    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < n; ++i) {
        if (i == 0) {
            J.col(0) = Slist[0];
        } else {
            Eigen::Matrix<double,6,6> AdT = adjoint(T);
            J.col(i) = AdT * Slist[i];
        }
        T = T * se3Exp(Slist[i], q(i));
    }
    return J;
}

MatrixXd jacobianBody(const std::vector<VectorXd>& Slist,
                      const Matrix4d& M,
                      const VectorXd& q) {
    Matrix4d T = fkPoeSpace(Slist, M, q);
    MatrixXd Js = jacobianSpace(Slist, q);
    Eigen::Matrix<double,6,6> AdInv = adjoint(T.inverse());
    return AdInv * Js;
}

int main() {
    double l1 = 1.0, l2 = 0.8;
    VectorXd S1(6), S2(6);
    S1 << 0, 0, 1, 0, 0, 0;
    S2 << 0, 0, 1, 0, -l1, 0;
    std::vector<VectorXd> Slist = {S1, S2};

    Matrix4d M = Matrix4d::Identity();
    M(0,3) = l1 + l2;

    VectorXd q(2);
    q << 0.5, -0.4;

    Matrix4d T = fkPoeSpace(Slist, M, q);
    MatrixXd Js = jacobianSpace(Slist, q);
    MatrixXd Jb = jacobianBody(Slist, M, q);

    std::cout << "T(q):\n" << T << std::endl;
    std::cout << "J_s(q):\n" << Js << std::endl;
    std::cout << "J_b(q):\n" << Jb << std::endl;
    return 0;
}
      
