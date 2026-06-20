// Chapter11_Lesson3.cpp
// Controllability in companion controllable canonical form.
// Build A_c, B_c, Q_c = [B, AB, ..., A^(n-1)B], and compute rank by Gaussian elimination.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix zeros(int rows, int cols) {
    return Matrix(rows, std::vector<double>(cols, 0.0));
}

Matrix identity(int n) {
    Matrix I = zeros(n, n);
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix matmul(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int m = static_cast<int>(A[0].size());
    int c = static_cast<int>(B[0].size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i) {
        for (int k = 0; k < m; ++k) {
            for (int j = 0; j < c; ++j) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

Matrix companionA(const std::vector<double>& coeffs) {
    int n = static_cast<int>(coeffs.size());
    Matrix A = zeros(n, n);
    for (int i = 0; i < n - 1; ++i) A[i][i + 1] = 1.0;
    for (int j = 0; j < n; ++j) A[n - 1][j] = -coeffs[j];
    return A;
}

Matrix companionB(int n) {
    Matrix B = zeros(n, 1);
    B[n - 1][0] = 1.0;
    return B;
}

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    Matrix Q = zeros(n, n);
    Matrix Ak = identity(n);
    for (int col = 0; col < n; ++col) {
        Matrix block = matmul(Ak, B);
        for (int row = 0; row < n; ++row) Q[row][col] = block[row][0];
        Ak = matmul(Ak, A);
    }
    return Q;
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
        double div = M[r][c];
        for (int j = c; j < cols; ++j) M[r][j] /= div;
        for (int i = 0; i < rows; ++i) {
            if (i == r) continue;
            double factor = M[i][c];
            for (int j = c; j < cols; ++j) M[i][j] -= factor * M[r][j];
        }
        ++r;
    }
    return r;
}

void printMatrix(const Matrix& M, const std::string& name) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double x : row) std::cout << std::setw(12) << x << " ";
        std::cout << "\n";
    }
}

int main() {
    std::vector<double> coeffs = {6.0, 11.0, 6.0}; // p(s)=s^3+6s^2+11s+6
    Matrix A = companionA(coeffs);
    Matrix B = companionB(static_cast<int>(coeffs.size()));
    Matrix Q = controllabilityMatrix(A, B);

    printMatrix(A, "A_c");
    printMatrix(B, "B_c");
    printMatrix(Q, "Q_c");

    std::cout << "rank(Q_c) = " << rank(Q) << "\n";
    std::cout << "The companion pair is controllable when rank(Q_c) equals n.\n";
    return 0;
}
