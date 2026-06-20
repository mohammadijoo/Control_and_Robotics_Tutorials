// Chapter14_Lesson4.cpp
// Observability in canonical (observable) forms from scratch.
// Compile: g++ -std=c++17 Chapter14_Lesson4.cpp -O2 -o Chapter14_Lesson4

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix zeros(int r, int c) {
    return Matrix(r, std::vector<double>(c, 0.0));
}

Matrix identity(int n) {
    Matrix I = zeros(n, n);
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

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

Matrix vstack(const std::vector<Matrix>& blocks) {
    int totalRows = 0;
    int cols = static_cast<int>(blocks[0][0].size());
    for (const auto& M : blocks) totalRows += static_cast<int>(M.size());
    Matrix R = zeros(totalRows, cols);
    int row = 0;
    for (const auto& M : blocks) {
        for (int i = 0; i < static_cast<int>(M.size()); ++i) {
            R[row++] = M[i];
        }
    }
    return R;
}

int rankGaussian(Matrix A, double tol = 1e-10) {
    int m = static_cast<int>(A.size());
    int n = static_cast<int>(A[0].size());
    int rank = 0;
    for (int col = 0; col < n && rank < m; ++col) {
        int pivot = rank;
        for (int i = rank + 1; i < m; ++i) {
            if (std::fabs(A[i][col]) > std::fabs(A[pivot][col])) pivot = i;
        }
        if (std::fabs(A[pivot][col]) <= tol) continue;
        std::swap(A[pivot], A[rank]);
        double div = A[rank][col];
        for (int j = col; j < n; ++j) A[rank][j] /= div;
        for (int i = 0; i < m; ++i) {
            if (i == rank) continue;
            double factor = A[i][col];
            for (int j = col; j < n; ++j) A[i][j] -= factor * A[rank][j];
        }
        ++rank;
    }
    return rank;
}

Matrix observableCompanion(const std::vector<double>& den) {
    // den = [a0, a1, ..., a_{n-1}]
    int n = static_cast<int>(den.size());
    Matrix A = zeros(n, n);
    for (int i = 0; i < n; ++i) A[i][0] = -den[n - 1 - i];
    for (int i = 0; i < n - 1; ++i) A[i][i + 1] = 1.0;
    return A;
}

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = static_cast<int>(A.size());
    std::vector<Matrix> rows;
    Matrix Ak = identity(n);
    for (int k = 0; k < n; ++k) {
        rows.push_back(multiply(C, Ak));
        Ak = multiply(Ak, A);
    }
    return vstack(rows);
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double v : row) std::cout << std::setw(12) << v << " ";
        std::cout << "\n";
    }
}

int main() {
    // Denominator: s^3 + 6s^2 + 11s + 6
    std::vector<double> den = {6.0, 11.0, 6.0};
    Matrix A = observableCompanion(den);
    Matrix C = {{1.0, 0.0, 0.0}};
    Matrix O = observabilityMatrix(A, C);

    printMatrix("A_o", A);
    printMatrix("C_o", C);
    printMatrix("O", O);
    std::cout << "rank(O) = " << rankGaussian(O) << " out of " << A.size() << "\n";

    return 0;
}
