// Chapter13_Lesson1.cpp
// From-scratch observability matrix and rank test for small LTI systems.
// Compile: g++ -std=c++17 Chapter13_Lesson1.cpp -o Chapter13_Lesson1

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix identity(int n) {
    Matrix I(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int m = static_cast<int>(A.size());
    int p = static_cast<int>(B.size());
    int n = static_cast<int>(B[0].size());
    Matrix C(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; ++i)
        for (int k = 0; k < p; ++k)
            for (int j = 0; j < n; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix vstack(const std::vector<Matrix>& blocks) {
    int cols = static_cast<int>(blocks[0][0].size());
    Matrix out;
    for (const auto& B : blocks) {
        for (const auto& row : B) {
            if (static_cast<int>(row.size()) != cols) {
                throw std::runtime_error("Column mismatch in vstack.");
            }
            out.push_back(row);
        }
    }
    return out;
}

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = static_cast<int>(A.size());
    Matrix Ak = identity(n);
    std::vector<Matrix> blocks;
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(C, Ak));
        Ak = multiply(Ak, A);
    }
    return vstack(blocks);
}

int rankGaussian(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int r = 0;
    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i)
            if (std::fabs(M[i][c]) > std::fabs(M[pivot][c])) pivot = i;

        if (std::fabs(M[pivot][c]) < tol) continue;

        std::swap(M[r], M[pivot]);
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

void printMatrix(const Matrix& M) {
    for (const auto& row : M) {
        for (double x : row) std::cout << std::setw(10) << std::setprecision(5) << x << " ";
        std::cout << "\n";
    }
}

void report(const std::string& name, const Matrix& A, const Matrix& C) {
    Matrix O = observabilityMatrix(A, C);
    std::cout << "\n" << name << "\nO_n =\n";
    printMatrix(O);
    std::cout << "rank(O_n) = " << rankGaussian(O)
              << " out of n = " << A.size() << "\n";
}

int main() {
    Matrix A1 = {{0.0, 1.0},
                 {-2.0, -3.0}};
    Matrix C1 = {{1.0, 0.0}};
    report("Example 1: position sensor, observable", A1, C1);

    Matrix A2 = {{-1.0, 0.0},
                 {0.0, -2.0}};
    Matrix C2 = {{1.0, 0.0}};
    report("Example 2: second state invisible, unobservable", A2, C2);

    return 0;
}
