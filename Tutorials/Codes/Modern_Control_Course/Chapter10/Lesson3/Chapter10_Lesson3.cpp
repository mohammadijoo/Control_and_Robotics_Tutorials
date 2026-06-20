// Chapter10_Lesson3.cpp
// Finite-time reachability and minimum-energy steering using Eigen.
// Compile example:
// g++ -std=c++17 Chapter10_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter10_Lesson3

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    const int n = A.rows();
    Matrix C(n, n * B.cols());
    Matrix Apow = Matrix::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        C.block(0, k * B.cols(), n, B.cols()) = Apow * B;
        Apow = Apow * A;
    }
    return C;
}

Matrix finiteTimeGramian(const Matrix& A, const Matrix& B, double T, int steps = 5000) {
    const int n = A.rows();
    Matrix W = Matrix::Zero(n, n);
    const double h = T / steps;
    for (int k = 0; k <= steps; ++k) {
        double s = k * h;
        double weight = (k == 0 || k == steps) ? 0.5 : 1.0;
        Matrix E = (A * s).exp();
        W += weight * (E * B * B.transpose() * E.transpose());
    }
    W *= h;
    return 0.5 * (W + W.transpose());
}

Matrix pseudoInverse(const Matrix& M, double tol = 1e-10) {
    Eigen::JacobiSVD<Matrix> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector s = svd.singularValues();
    Matrix Sinv = Matrix::Zero(svd.matrixV().cols(), svd.matrixU().cols());
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) {
            Sinv(i, i) = 1.0 / s(i);
        }
    }
    return svd.matrixV() * Sinv * svd.matrixU().transpose();
}

Vector minimumEnergyInput(const Matrix& A, const Matrix& B, const Matrix& Winv,
                          const Vector& d, double T, double t) {
    Matrix E = (A.transpose() * (T - t)).exp();
    return B.transpose() * E * Winv * d;
}

int main() {
    Matrix A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;
    Matrix B(2, 1);
    B << 0.0,
         1.0;
    Vector x0(2), xT(2);
    x0 << 0.0, 0.0;
    xT << 1.0, 0.0;

    Matrix C = controllabilityMatrix(A, B);
    Eigen::FullPivLU<Matrix> lu(C);
    std::cout << "Controllability matrix:\n" << C << "\n";
    std::cout << "rank = " << lu.rank() << "\n\n";

    for (double T : {0.5, 1.0, 2.0, 4.0}) {
        Matrix PhiT = (A * T).exp();
        Matrix W = finiteTimeGramian(A, B, T);
        Matrix Winv = pseudoInverse(W);
        Vector d = xT - PhiT * x0;
        double energy = d.transpose() * Winv * d;

        std::cout << "T = " << T << "\n";
        std::cout << "W(T):\n" << W << "\n";
        std::cout << "det(W) = " << W.determinant() << "\n";
        std::cout << "minimum energy = " << energy << "\n";
        std::cout << "u(0) = " << minimumEnergyInput(A, B, Winv, d, T, 0.0).transpose() << "\n";
        std::cout << "u(T/2) = " << minimumEnergyInput(A, B, Winv, d, T, T / 2.0).transpose() << "\n";
        std::cout << "u(T) = " << minimumEnergyInput(A, B, Winv, d, T, T).transpose() << "\n\n";
    }
    return 0;
}
