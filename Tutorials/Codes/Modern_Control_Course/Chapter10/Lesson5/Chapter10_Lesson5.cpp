// Chapter10_Lesson5.cpp
// Modern Control - Chapter 10, Lesson 5
// Controllability examples using a small from-scratch matrix implementation.
// Compile:
//     g++ -std=c++17 Chapter10_Lesson5.cpp -o Chapter10_Lesson5
// Run:
//     ./Chapter10_Lesson5

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
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

int rankGaussian(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int rank = 0;

    for (int col = 0; col < cols && rank < rows; ++col) {
        int pivot = rank;
        for (int r = rank + 1; r < rows; ++r) {
            if (std::fabs(M[r][col]) > std::fabs(M[pivot][col])) {
                pivot = r;
            }
        }

        if (std::fabs(M[pivot][col]) <= tol) continue;

        std::swap(M[rank], M[pivot]);
        double pivotValue = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= pivotValue;

        for (int r = 0; r < rows; ++r) {
            if (r == rank) continue;
            double factor = M[r][col];
            for (int j = col; j < cols; ++j) {
                M[r][j] -= factor * M[rank][j];
            }
        }
        ++rank;
    }
    return rank;
}

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    std::vector<Matrix> blocks;
    Matrix Ak = identity(n);
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(Ak, B));
        Ak = multiply(Ak, A);
    }
    return hstack(blocks);
}

void printMatrix(const Matrix& M) {
    for (const auto& row : M) {
        for (double value : row) {
            std::cout << std::setw(10) << std::setprecision(4) << value << " ";
        }
        std::cout << "\n";
    }
}

void analyze(const std::string& name, const Matrix& A, const Matrix& B) {
    Matrix C = controllabilityMatrix(A, B);
    int r = rankGaussian(C);
    int n = static_cast<int>(A.size());

    std::cout << "\n" << name << "\n";
    std::cout << std::string(name.size(), '-') << "\n";
    std::cout << "Controllability matrix:\n";
    printMatrix(C);
    std::cout << "rank(C) = " << r << " out of n = " << n << "\n";
    std::cout << "Conclusion: " << (r == n ? "controllable" : "uncontrollable") << "\n";
}

int main() {
    Matrix A1 = {{0, 1},
                 {0, 0}};
    Matrix B1 = {{0},
                 {1}};
    analyze("Example 1: double integrator", A1, B1);

    Matrix A2 = {{0, 1, 0, 0},
                 {0, 0, 0, 0},
                 {0, 0, 0, 1},
                 {0, 0, 0, 0}};
    Matrix B2 = {{0},
                 {1},
                 {0},
                 {0}};
    analyze("Example 2: two decoupled masses, one actuator", A2, B2);

    double m1 = 1.0, m2 = 1.0, k1 = 1.0, k2 = 1.2, kc = 0.8, c1 = 0.1, c2 = 0.2;
    Matrix A3 = {{0, 1, 0, 0},
                 {-(k1 + kc) / m1, -c1 / m1, kc / m1, 0},
                 {0, 0, 0, 1},
                 {kc / m2, 0, -(k2 + kc) / m2, -c2 / m2}};
    Matrix B3 = {{0},
                 {1.0 / m1},
                 {0},
                 {0}};
    analyze("Example 3: coupled two-mass oscillator", A3, B3);

    Matrix A4 = {{-1, 0, 0},
                 {0, -2, 0},
                 {0, 0, -3}};
    Matrix B4 = {{1},
                 {0},
                 {1}};
    analyze("Example 4: diagonal system with one unactuated mode", A4, B4);

    return 0;
}
