// Chapter19_Lesson5.cpp
// Physical Interpretation of Decomposed Subsystems in Kalman Decomposition
// Self-contained C++17 implementation: matrix products, powers, and rank.

#include <cmath>
#include <iostream>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix zeros(int r, int c) { return Matrix(r, std::vector<double>(c, 0.0)); }
Matrix eye(int n) { Matrix I = zeros(n, n); for (int i = 0; i < n; ++i) I[i][i] = 1.0; return I; }

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = (int)A.size(), m = (int)A[0].size(), c = (int)B[0].size();
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int k = 0; k < m; ++k)
            for (int j = 0; j < c; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int r = (int)blocks[0].size(), ctotal = 0;
    for (const auto& M : blocks) ctotal += (int)M[0].size();
    Matrix H = zeros(r, ctotal);
    int offset = 0;
    for (const auto& M : blocks) {
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < (int)M[0].size(); ++j)
                H[i][offset + j] = M[i][j];
        offset += (int)M[0].size();
    }
    return H;
}

Matrix vstack(const std::vector<Matrix>& blocks) {
    int c = (int)blocks[0][0].size(), rtotal = 0;
    for (const auto& M : blocks) rtotal += (int)M.size();
    Matrix V = zeros(rtotal, c);
    int offset = 0;
    for (const auto& M : blocks) {
        for (int i = 0; i < (int)M.size(); ++i)
            for (int j = 0; j < c; ++j)
                V[offset + i][j] = M[i][j];
        offset += (int)M.size();
    }
    return V;
}

int rank(Matrix M, double tol = 1e-9) {
    int rows = (int)M.size(), cols = (int)M[0].size(), r = 0;
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

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = (int)A.size();
    std::vector<Matrix> blocks;
    Matrix Ak = eye(n);
    for (int k = 0; k < n; ++k) { blocks.push_back(multiply(Ak, B)); Ak = multiply(Ak, A); }
    return hstack(blocks);
}

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = (int)A.size();
    std::vector<Matrix> blocks;
    Matrix Ak = eye(n);
    for (int k = 0; k < n; ++k) { blocks.push_back(multiply(C, Ak)); Ak = multiply(Ak, A); }
    return vstack(blocks);
}

int main() {
    Matrix A = {
        {0.0, 1.0, 0.0, 0.2, 0.0},
        {-2.0, -3.0, 0.0, 0.0, 0.0},
        {0.0, 0.3, -4.0, 0.1, 0.0},
        {0.0, 0.0, 0.0, -0.5, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.2}
    };
    Matrix B = {{0.0}, {1.0}, {1.0}, {0.0}, {0.0}};
    Matrix C = {{1.0, 0.0, 0.0, 1.0, 0.0}};

    Matrix Wc = controllabilityMatrix(A, B);
    Matrix Wo = observabilityMatrix(A, C);
    std::cout << "rank(Wc) = " << rank(Wc) << " out of 5\n";
    std::cout << "rank(Wo) = " << rank(Wo) << " out of 5\n";
    std::cout << "Reachable states: co plus c_no. Observable states: co plus no_o.\n";
    std::cout << "Only the co block contributes to the zero-initial transfer function.\n";
    return 0;
}
