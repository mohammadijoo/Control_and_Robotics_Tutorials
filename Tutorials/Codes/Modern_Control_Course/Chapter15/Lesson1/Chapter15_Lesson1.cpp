// Chapter15_Lesson1.cpp
// Finite-horizon observability Gramian by integrating:
//   dW/dt = A^T W + W A + C^T C, W(0)=0.
//
// Library used:
//   Eigen 3
//
// Compile, for example:
//   g++ Chapter15_Lesson1.cpp -std=c++17 -O2 -I /path/to/eigen -o Chapter15_Lesson1

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Matrix = Eigen::MatrixXd;

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    const int n = A.rows();
    std::vector<Matrix> blocks;
    Matrix Ak = Matrix::Identity(n, n);

    for (int k = 0; k < n; ++k) {
        blocks.push_back(C * Ak);
        Ak = Ak * A;
    }

    Matrix O(C.rows() * n, n);
    for (int k = 0; k < n; ++k) {
        O.block(k * C.rows(), 0, C.rows(), n) = blocks[k];
    }
    return O;
}

Matrix gramianRhs(const Matrix& A, const Matrix& Q, const Matrix& W) {
    return A.transpose() * W + W * A + Q;
}

Matrix finiteObservabilityGramianRK4(const Matrix& A, const Matrix& C, double T, int steps) {
    const int n = A.rows();
    const double h = T / static_cast<double>(steps);
    const Matrix Q = C.transpose() * C;
    Matrix W = Matrix::Zero(n, n);

    for (int k = 0; k < steps; ++k) {
        Matrix K1 = gramianRhs(A, Q, W);
        Matrix K2 = gramianRhs(A, Q, W + 0.5 * h * K1);
        Matrix K3 = gramianRhs(A, Q, W + 0.5 * h * K2);
        Matrix K4 = gramianRhs(A, Q, W + h * K3);
        W += (h / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
    }

    return 0.5 * (W + W.transpose());
}

int main() {
    Matrix A(2, 2);
    A << 0.0, 1.0,
        -2.0, -3.0;

    Matrix C(1, 2);
    C << 1.0, 0.0;

    Matrix O = observabilityMatrix(A, C);
    std::cout << "Observability matrix O:\n" << O << "\n\n";
    std::cout << "rank(O) = " << O.fullPivLu().rank() << "\n\n";

    const double T = 4.0;
    Matrix W = finiteObservabilityGramianRK4(A, C, T, 20000);

    std::cout << "Finite-horizon observability Gramian W_o(T):\n" << W << "\n\n";

    Eigen::SelfAdjointEigenSolver<Matrix> solver(W);
    std::cout << "Eigenvalues of W_o(T):\n" << solver.eigenvalues() << "\n\n";

    Eigen::Vector2d x0;
    x0 << 1.0, -0.5;
    double energy = (x0.transpose() * W * x0)(0, 0);
    std::cout << "Output energy x0^T W_o(T) x0 = " << energy << "\n";

    return 0;
}
