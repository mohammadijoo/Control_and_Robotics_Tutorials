/*
Chapter16_Lesson5.cpp
Portable C++ implementation for CCF construction, controllability rank,
and direct pole-placement gain in controllable canonical form.

Production libraries for larger Modern Control programs:
  - Eigen, Armadillo, Boost.uBLAS, SLICOT wrappers
*/

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

Matrix zeros(int r, int c) { return Matrix(r, Vector(c, 0.0)); }

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int m = static_cast<int>(A[0].size());
    int c = static_cast<int>(B[0].size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int k = 0; k < m; ++k)
            for (int j = 0; j < c; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix companionCCF(const Vector& aAscending) {
    int n = static_cast<int>(aAscending.size());
    Matrix A = zeros(n, n);
    for (int i = 0; i < n - 1; ++i) A[i][i + 1] = 1.0;
    for (int j = 0; j < n; ++j) A[n - 1][j] = -aAscending[j];
    return A;
}

Matrix inputB(int n) {
    Matrix B = zeros(n, 1);
    B[n - 1][0] = 1.0;
    return B;
}

Matrix controllabilityMatrix(Matrix A, Matrix B) {
    int n = static_cast<int>(A.size());
    Matrix Q = zeros(n, n);
    Matrix AkB = B;
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) Q[i][k] = AkB[i][0];
        AkB = multiply(A, AkB);
    }
    return Q;
}

int rankGaussian(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int rank = 0;
    for (int col = 0; col < cols && rank < rows; ++col) {
        int pivot = rank;
        for (int i = rank + 1; i < rows; ++i)
            if (std::fabs(M[i][col]) > std::fabs(M[pivot][col])) pivot = i;
        if (std::fabs(M[pivot][col]) <= tol) continue;
        std::swap(M[pivot], M[rank]);
        double div = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= div;
        for (int i = 0; i < rows; ++i) {
            if (i == rank) continue;
            double factor = M[i][col];
            for (int j = col; j < cols; ++j) M[i][j] -= factor * M[rank][j];
        }
        ++rank;
    }
    return rank;
}

Vector desiredAscendingForExample() {
    // Desired poles -2, -3, -4:
    // (s+2)(s+3)(s+4) = s^3 + 9s^2 + 26s + 24
    return {24.0, 26.0, 9.0};
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double v : row) std::cout << std::setw(12) << v << " ";
        std::cout << "\n";
    }
}

int main() {
    Vector a = {6.0, 11.0, 6.0}; // a0, a1, a2 for s^3 + 6s^2 + 11s + 6
    Matrix A = companionCCF(a);
    Matrix B = inputB(static_cast<int>(a.size()));
    Matrix Q = controllabilityMatrix(A, B);

    printMatrix("A_c", A);
    printMatrix("B_c", B);
    printMatrix("Q_c", Q);
    std::cout << "rank(Q_c) = " << rankGaussian(Q) << "\n";

    Vector alpha = desiredAscendingForExample();
    Vector K(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) K[i] = alpha[i] - a[i];

    std::cout << "K = [ ";
    for (double k : K) std::cout << k << " ";
    std::cout << "]\n";
    return 0;
}
