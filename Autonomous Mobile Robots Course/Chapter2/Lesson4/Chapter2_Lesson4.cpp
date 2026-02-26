// Chapter2_Lesson4.cpp
// Autonomous Mobile Robots — Chapter 2 Lesson 4
// Omnidirectional Bases (mecanum, Swedish wheels)
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter2_Lesson4.cpp -I /path/to/eigen -o Chapter2_Lesson4
//
// Dependencies:
//   - Eigen (header-only) for linear algebra

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

static Eigen::MatrixXd pinv_svd(const Eigen::MatrixXd& A, double tol = 1e-10) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd s = svd.singularValues();
    Eigen::VectorXd s_inv(s.size());
    for (int i = 0; i < s.size(); ++i) {
        s_inv(i) = (s(i) > tol) ? (1.0 / s(i)) : 0.0;
    }
    return svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().transpose();
}

struct Mecanum4 {
    double r;
    double lx;
    double ly;
    double alpha; // rad

    Eigen::Matrix<double, 4, 3> J() const {
        const double a = lx + ly;
        const double kappa = r * std::cos(alpha);
        Eigen::Matrix<double, 4, 3> A;
        A <<  1.0, -1.0, -a,
              1.0,  1.0,  a,
              1.0,  1.0, -a,
              1.0, -1.0,  a;
        return (1.0 / kappa) * A;
    }

    Eigen::Vector4d inverse(double vx, double vy, double omega) const {
        Eigen::Vector3d v(vx, vy, omega);
        return J() * v;
    }

    Eigen::Vector3d forward_ls(const Eigen::Vector4d& w) const {
        Eigen::MatrixXd Jd = J();
        Eigen::MatrixXd Jp = pinv_svd(Jd);
        return Jp * w;
    }
};

int main() {
    Mecanum4 mec;
    mec.r = 0.05;
    mec.lx = 0.20;
    mec.ly = 0.15;
    mec.alpha = M_PI / 4.0;

    Eigen::Vector3d v_cmd(0.40, -0.10, 0.60);
    Eigen::Vector4d w = mec.inverse(v_cmd(0), v_cmd(1), v_cmd(2));
    Eigen::Vector3d v_hat = mec.forward_ls(w);

    std::cout << "Commanded twist [vx vy omega]^T =\n" << v_cmd << "\n\n";
    std::cout << "Wheel speeds [w1..w4]^T (rad/s)=\n" << w << "\n\n";
    std::cout << "Reconstructed twist (LS)=\n" << v_hat << "\n";

    return 0;
}
