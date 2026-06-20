/*
Chapter22_Lesson4.cpp
Requirements for Implementing State Feedback

This standalone C++ example checks rank controllability, computes closed-loop
eigenvalues for a supplied gain K, and simulates x_dot = (A - BK)x.

Compile:
  g++ -std=c++17 Chapter22_Lesson4.cpp -o Chapter22_Lesson4
*/

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int c = static_cast<int>(B[0].size());
    int inner = static_cast<int>(B.size());
    Matrix C(r, std::vector<double>(c, 0.0));
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            for (int k = 0; k < inner; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

Matrix subtract(const Matrix& A, const Matrix& B) {
    Matrix C = A;
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            C[i][j] -= B[i][j];
        }
    }
    return C;
}

int rank(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int r = 0;

    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i) {
            if (std::fabs(M[i][c]) > std::fabs(M[pivot][c])) pivot = i;
        }
        if (std::fabs(M[pivot][c]) <= tol) continue;

        std::swap(M[pivot], M[r]);
        double piv = M[r][c];
        for (int j = c; j < cols; ++j) M[r][j] /= piv;

        for (int i = 0; i < rows; ++i) {
            if (i == r) continue;
            double factor = M[i][c];
            for (int j = c; j < cols; ++j) M[i][j] -= factor * M[r][j];
        }
        ++r;
    }
    return r;
}

Matrix controllabilityMatrix2x2(const Matrix& A, const Matrix& B) {
    Matrix AB = multiply(A, B);
    return {{B[0][0], AB[0][0]},
            {B[1][0], AB[1][0]}};
}

std::pair<std::complex<double>, std::complex<double>> eigenvalues2x2(const Matrix& A) {
    double tr = A[0][0] + A[1][1];
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    std::complex<double> disc = std::complex<double>(tr * tr - 4.0 * det, 0.0);
    std::complex<double> root = std::sqrt(disc);
    return {(tr + root) / 2.0, (tr - root) / 2.0};
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << "\n";
    for (const auto& row : M) {
        for (double v : row) std::cout << std::setw(12) << v << " ";
        std::cout << "\n";
    }
}

int main() {
    Matrix A = {{0.0, 1.0},
                {-2.0, -3.0}};
    Matrix B = {{0.0},
                {1.0}};
    Matrix K = {{4.0, 2.0}};

    Matrix Wc = controllabilityMatrix2x2(A, B);
    printMatrix("Controllability matrix Wc:", Wc);
    std::cout << "rank(Wc) = " << rank(Wc) << " required = 2\n";

    Matrix BK = multiply(B, K);
    Matrix Acl = subtract(A, BK);
    printMatrix("\nClosed-loop matrix Acl = A - BK:", Acl);

    auto eigs = eigenvalues2x2(Acl);
    std::cout << "closed-loop eigenvalues: " << eigs.first << ", " << eigs.second << "\n";

    std::vector<double> x = {1.0, 0.0};
    double dt = 0.01;
    double tf = 5.0;
    int steps = static_cast<int>(tf / dt);

    for (int k = 0; k < steps; ++k) {
        std::vector<double> dx = {
            Acl[0][0] * x[0] + Acl[0][1] * x[1],
            Acl[1][0] * x[0] + Acl[1][1] * x[1]
        };
        x[0] += dt * dx[0];
        x[1] += dt * dx[1];
    }

    std::cout << "Final simulated state: [" << x[0] << ", " << x[1] << "]\n";
    return 0;
}
