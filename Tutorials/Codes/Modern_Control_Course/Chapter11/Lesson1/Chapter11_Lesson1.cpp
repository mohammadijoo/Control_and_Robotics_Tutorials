// Chapter11_Lesson1.cpp
// Kalman controllability matrix and rank condition using only the C++ standard library.

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
    int k = static_cast<int>(A[0].size());
    int c = static_cast<int>(B[0].size());
    if (static_cast<int>(B.size()) != k) {
        throw std::invalid_argument("Inner dimensions do not match.");
    }
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            for (int p = 0; p < k; ++p) {
                C[i][j] += A[i][p] * B[p][j];
            }
        }
    }
    return C;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int rows = static_cast<int>(blocks[0].size());
    int totalCols = 0;
    for (const auto& M : blocks) totalCols += static_cast<int>(M[0].size());
    Matrix H = zeros(rows, totalCols);

    int offset = 0;
    for (const auto& M : blocks) {
        int cols = static_cast<int>(M[0].size());
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                H[i][offset + j] = M[i][j];
        offset += cols;
    }
    return H;
}

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    if (n == 0 || static_cast<int>(A[0].size()) != n) {
        throw std::invalid_argument("A must be square.");
    }
    if (static_cast<int>(B.size()) != n) {
        throw std::invalid_argument("B must have the same number of rows as A.");
    }

    std::vector<Matrix> blocks;
    Matrix Ak = identity(n);
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(Ak, B));
        Ak = multiply(A, Ak);
    }
    return hstack(blocks);
}

int rankGaussian(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int rank = 0;

    for (int col = 0; col < cols && rank < rows; ++col) {
        int pivot = rank;
        for (int i = rank + 1; i < rows; ++i) {
            if (std::fabs(M[i][col]) > std::fabs(M[pivot][col])) pivot = i;
        }

        if (std::fabs(M[pivot][col]) <= tol) continue;
        std::swap(M[rank], M[pivot]);

        double piv = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= piv;

        for (int i = 0; i < rows; ++i) {
            if (i == rank) continue;
            double factor = M[i][col];
            for (int j = col; j < cols; ++j) M[i][j] -= factor * M[rank][j];
        }
        ++rank;
    }
    return rank;
}

void printMatrix(const Matrix& M) {
    for (const auto& row : M) {
        for (double v : row) {
            std::cout << std::setw(12) << std::setprecision(6) << v << " ";
        }
        std::cout << "\n";
    }
}

int main() {
    Matrix A = {{0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0},
                {-6.0, -11.0, -6.0}};
    Matrix B = {{0.0},
                {0.0},
                {1.0}};

    Matrix Ck = controllabilityMatrix(A, B);
    int r = rankGaussian(Ck);

    std::cout << "Kalman controllability matrix C_K:\n";
    printMatrix(Ck);
    std::cout << "rank(C_K) = " << r << "\n";
    std::cout << "controllable = " << (r == static_cast<int>(A.size()) ? "true" : "false") << "\n";

    return 0;
}
