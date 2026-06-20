/*
Chapter17_Lesson3.cpp
Diagonal modal form for a continuous-time LTI system with distinct real eigenvalues.

Required library:
    Eigen 3  (https://eigen.tuxfamily.org)
Compile example:
    g++ Chapter17_Lesson3.cpp -std=c++17 -I /path/to/eigen -O2 -o modal_form
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iostream>

int main() {
    using Matrix = Eigen::MatrixXd;
    using CMatrix = Eigen::MatrixXcd;
    using CVector = Eigen::VectorXcd;

    Matrix A(3, 3);
    A << 0.0, 1.0, 0.0,
        -2.0, -3.0, 0.0,
         0.5, 0.0, -4.0;

    Matrix B(3, 1);
    B << 0.0, 1.0, 0.0;

    Matrix C(1, 3);
    C << 1.0, 0.0, 1.0;

    Eigen::EigenSolver<Matrix> solver(A);
    CVector lambda = solver.eigenvalues();
    CMatrix V = solver.eigenvectors();
    CMatrix Vinv = V.inverse();
    CMatrix Lambda = Vinv * A.cast<std::complex<double>>() * V;
    CMatrix Bm = Vinv * B.cast<std::complex<double>>();
    CMatrix Cm = C.cast<std::complex<double>>() * V;

    std::cout << "Eigenvalues:\n" << lambda << "\n\n";
    std::cout << "Modal A matrix Lambda = V^{-1} A V:\n" << Lambda << "\n\n";
    std::cout << "Modal input matrix Bm = V^{-1} B:\n" << Bm << "\n\n";
    std::cout << "Modal output matrix Cm = C V:\n" << Cm << "\n\n";

    CVector x0(3);
    x0 << 1.0, 0.0, -0.5;
    CVector z0 = Vinv * x0;

    std::cout << "Zero-input modal response y(t):\n";
    for (double t = 0.0; t <= 5.0; t += 1.0) {
        CMatrix expLambda = CMatrix::Zero(3, 3);
        for (int i = 0; i < 3; ++i) {
            expLambda(i, i) = std::exp(lambda(i) * t);
        }
        CVector zt = expLambda * z0;
        CVector xt = V * zt;
        std::complex<double> yt = (C.cast<std::complex<double>>() * xt)(0);
        std::cout << "t=" << t << ", y=" << yt << "\n";
    }

    return 0;
}
