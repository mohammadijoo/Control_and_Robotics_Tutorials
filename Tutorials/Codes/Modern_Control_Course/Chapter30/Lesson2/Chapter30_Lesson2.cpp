/*
Chapter30_Lesson2.cpp
Coordinate scaling and conditioning for a state-space realization.

Dependencies:
    Eigen 3.4 or later
Compile example:
    g++ -std=c++17 Chapter30_Lesson2.cpp -I /path/to/eigen -O2 -o Chapter30_Lesson2
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iomanip>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = static_cast<int>(A.rows());
    MatrixXd Mc(n, n * B.cols());
    MatrixXd Apow = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Mc.block(0, k * B.cols(), n, B.cols()) = Apow * B;
        Apow = Apow * A;
    }
    return Mc;
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = static_cast<int>(A.rows());
    MatrixXd Mo(n * C.rows(), n);
    MatrixXd Apow = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Mo.block(k * C.rows(), 0, C.rows(), n) = C * Apow;
        Apow = Apow * A;
    }
    return Mo;
}

double conditionNumber2(const MatrixXd& M) {
    Eigen::JacobiSVD<MatrixXd> svd(M);
    auto s = svd.singularValues();
    return s(0) / s(s.size() - 1);
}

int main() {
    MatrixXd A(3, 3);
    A << 0.0,      1.0,      0.0,
        -2.0e3,  -5.0e1,   8.0e4,
         0.0,    -2.0e-2, -4.0e3;

    MatrixXd B(3, 1);
    B << 0.0, 0.0, 2.0e3;

    MatrixXd C(1, 3);
    C << 1.0, 0.0, 0.0;

    VectorXd xNom(3);
    xNom << 1.0e-3, 1.0e-1, 1.0e1;

    MatrixXd S = xNom.asDiagonal();
    MatrixXd Sinv = xNom.cwiseInverse().asDiagonal();

    MatrixXd Az = Sinv * A * S;
    MatrixXd Bz = Sinv * B;
    MatrixXd Cz = C * S;

    Eigen::EigenSolver<MatrixXd> eigA(A);
    Eigen::EigenSolver<MatrixXd> eigAz(Az);

    std::cout << std::scientific << std::setprecision(6);
    std::cout << "Eigenvalues of A:\n" << eigA.eigenvalues() << "\n\n";
    std::cout << "Eigenvalues of Az:\n" << eigAz.eigenvalues() << "\n\n";

    MatrixXd Mc = controllabilityMatrix(A, B);
    MatrixXd Mcz = controllabilityMatrix(Az, Bz);
    MatrixXd Mo = observabilityMatrix(A, C);
    MatrixXd Moz = observabilityMatrix(Az, Cz);

    std::cout << "cond(A)                = " << conditionNumber2(A) << "\n";
    std::cout << "cond(Az)               = " << conditionNumber2(Az) << "\n";
    std::cout << "cond(Mc)               = " << conditionNumber2(Mc) << "\n";
    std::cout << "cond(Mc scaled)        = " << conditionNumber2(Mcz) << "\n";
    std::cout << "cond(Mo)               = " << conditionNumber2(Mo) << "\n";
    std::cout << "cond(Mo scaled)        = " << conditionNumber2(Moz) << "\n";

    std::cout << "\nScaled realization matrices:\n";
    std::cout << "Az =\n" << Az << "\n";
    std::cout << "Bz =\n" << Bz << "\n";
    std::cout << "Cz =\n" << Cz << "\n";

    return 0;
}
