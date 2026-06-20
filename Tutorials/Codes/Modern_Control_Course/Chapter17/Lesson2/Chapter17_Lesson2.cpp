// Chapter17_Lesson2.cpp
// CCF-OCF duality using Eigen.
// Compile example: g++ -std=c++17 Chapter17_Lesson2.cpp -I /path/to/eigen -o Chapter17_Lesson2

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd ctrb(const MatrixXd& A, const MatrixXd& B) {
    int n = A.rows();
    MatrixXd Q(n, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Q.col(k) = Ak * B;
        Ak = Ak * A;
    }
    return Q;
}

MatrixXd obsv(const MatrixXd& A, const MatrixXd& C) {
    int n = A.rows();
    MatrixXd O(n, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        O.row(k) = C * Ak;
        Ak = Ak * A;
    }
    return O;
}

double transferValue(const MatrixXd& A, const MatrixXd& B,
                     const MatrixXd& C, double D, double s) {
    int n = A.rows();
    MatrixXd M = s * MatrixXd::Identity(n, n) - A;
    MatrixXd x = M.fullPivLu().solve(B);
    return (C * x)(0, 0) + D;
}

int main() {
    // G(s) = (2s^2 + 5s + 3)/(s^3 + 4s^2 + 6s + 4)
    int n = 3;
    VectorXd a(n); a << 4.0, 6.0, 4.0; // [a0,a1,a2]
    VectorXd b(n); b << 3.0, 5.0, 2.0; // [b0,b1,b2]

    MatrixXd Ac = MatrixXd::Zero(n, n);
    Ac.block(0, 1, n-1, n-1) = MatrixXd::Identity(n-1, n-1);
    Ac.row(n-1) = -a.transpose();

    MatrixXd Bc = MatrixXd::Zero(n, 1);
    Bc(n-1, 0) = 1.0;

    MatrixXd Cc(1, n);
    Cc = b.transpose();

    MatrixXd Ao = Ac.transpose();
    MatrixXd Bo = Cc.transpose();
    MatrixXd Co = Bc.transpose();

    std::cout << "Ac =\n" << Ac << "\n\n";
    std::cout << "Ao = Ac^T =\n" << Ao << "\n\n";

    MatrixXd Qc = ctrb(Ac, Bc);
    MatrixXd Oo = obsv(Ao, Co);

    std::cout << "rank ctrb(Ac,Bc) = " << Qc.fullPivLu().rank() << "\n";
    std::cout << "rank obsv(Ao,Co) = " << Oo.fullPivLu().rank() << "\n";
    std::cout << "||Oo - Qc^T||_F = " << (Oo - Qc.transpose()).norm() << "\n\n";

    for (double s : {0.5, 1.0, 2.0, 3.0}) {
        double hc = transferValue(Ac, Bc, Cc, 0.0, s);
        double ho = transferValue(Ao, Bo, Co, 0.0, s);
        std::cout << "s=" << s << " H_CCF=" << hc
                  << " H_OCF=" << ho << " error=" << std::abs(hc-ho) << "\n";
    }
}
