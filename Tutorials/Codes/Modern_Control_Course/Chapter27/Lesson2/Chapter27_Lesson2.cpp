// Chapter27_Lesson2.cpp
// Feedforward gain design using Eigen.
// Compile example: g++ Chapter27_Lesson2.cpp -I /path/to/eigen -O2 -std=c++17 -o Chapter27_Lesson2

#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>

using Matrix = Eigen::MatrixXd;

Matrix feedforwardFromDCGain(const Matrix& A, const Matrix& B, const Matrix& C,
                             const Matrix& D, const Matrix& K) {
    Matrix Acl = A - B * K;
    Matrix Ccl = C - D * K;
    Matrix G0 = D - Ccl * Acl.fullPivLu().solve(B);

    if (G0.rows() != G0.cols()) {
        throw std::runtime_error("G0 must be square for inversion.");
    }
    if (G0.fullPivLu().rank() < G0.rows()) {
        throw std::runtime_error("G0 is singular; static feedforward cannot give exact tracking.");
    }
    return G0.inverse();
}

Matrix feedforwardFromRegulatorEquations(const Matrix& A, const Matrix& B,
                                          const Matrix& C, const Matrix& D,
                                          const Matrix& K) {
    const int n = A.rows();
    const int m = B.cols();
    const int p = C.rows();

    Matrix R(n + p, n + m);
    R << A, B,
         C, D;

    Matrix RHS = Matrix::Zero(n + p, p);
    RHS.block(n, 0, p, p) = Matrix::Identity(p, p);

    if (R.fullPivLu().rank() < n + p) {
        throw std::runtime_error("Rosenbrock matrix at s=0 has deficient rank.");
    }

    Matrix Sol = R.completeOrthogonalDecomposition().solve(RHS);
    Matrix X = Sol.block(0, 0, n, p);
    Matrix U = Sol.block(n, 0, m, p);
    return U + K * X;
}

int main() {
    Matrix A(2, 2), B(2, 1), C(1, 2), D(1, 1), K(1, 2);
    A << 0.0, 1.0,
         0.0, 0.0;
    B << 0.0,
         1.0;
    C << 1.0, 0.0;
    D << 0.0;
    K << 4.0, 4.0;

    Matrix N1 = feedforwardFromDCGain(A, B, C, D, K);
    Matrix N2 = feedforwardFromRegulatorEquations(A, B, C, D, K);

    std::cout << "N from DC gain:\n" << N1 << "\n\n";
    std::cout << "N from regulator equations:\n" << N2 << "\n\n";

    // Euler simulation for a unit step reference.
    double dt = 0.001;
    double tf = 6.0;
    Matrix x = Matrix::Zero(2, 1);
    Matrix r = Matrix::Ones(1, 1);
    for (int kstep = 0; kstep < static_cast<int>(tf / dt); ++kstep) {
        Matrix u = -K * x + N1 * r;
        Matrix dx = A * x + B * u;
        x += dt * dx;
    }
    Matrix y = C * x + D * (-K * x + N1 * r);
    std::cout << "Final output y(tf):\n" << y << "\n";
    return 0;
}
