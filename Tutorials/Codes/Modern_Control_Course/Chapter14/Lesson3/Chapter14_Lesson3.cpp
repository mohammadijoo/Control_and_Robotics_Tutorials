// Chapter14_Lesson3.cpp
// Duality Between Controllability and Observability
// Compile: g++ -std=c++17 Chapter14_Lesson3.cpp -o Chapter14_Lesson3

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
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
        throw std::runtime_error("Dimension mismatch in multiply.");
    }
    Matrix M = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            for (int t = 0; t < k; ++t)
                M[i][j] += A[i][t] * B[t][j];
    return M;
}

Matrix transpose(const Matrix& A) {
    int r = static_cast<int>(A.size());
    int c = static_cast<int>(A[0].size());
    Matrix T = zeros(c, r);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            T[j][i] = A[i][j];
    return T;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int rows = static_cast<int>(blocks[0].size());
    int totalCols = 0;
    for (const auto& B : blocks) totalCols += static_cast<int>(B[0].size());
    Matrix H = zeros(rows, totalCols);
    int col0 = 0;
    for (const auto& B : blocks) {
        int cols = static_cast<int>(B[0].size());
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                H[i][col0 + j] = B[i][j];
        col0 += cols;
    }
    return H;
}

Matrix vstack(const std::vector<Matrix>& blocks) {
    int cols = static_cast<int>(blocks[0][0].size());
    int totalRows = 0;
    for (const auto& B : blocks) totalRows += static_cast<int>(B.size());
    Matrix V = zeros(totalRows, cols);
    int row0 = 0;
    for (const auto& B : blocks) {
        int rows = static_cast<int>(B.size());
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                V[row0 + i][j] = B[i][j];
        row0 += rows;
    }
    return V;
}

int rank(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int r = 0;
    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i)
            if (std::fabs(M[i][c]) > std::fabs(M[pivot][c])) pivot = i;

        if (std::fabs(M[pivot][c]) <= tol) continue;
        std::swap(M[pivot], M[r]);

        double div = M[r][c];
        for (int j = c; j < cols; ++j) M[r][j] /= div;

        for (int i = 0; i < rows; ++i) {
            if (i == r) continue;
            double factor = M[i][c];
            for (int j = c; j < cols; ++j)
                M[i][j] -= factor * M[r][j];
        }
        ++r;
    }
    return r;
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

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = static_cast<int>(A.size());
    std::vector<Matrix> blocks;
    Matrix Ak = identity(n);
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(C, Ak));
        Ak = multiply(Ak, A);
    }
    return vstack(blocks);
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double x : row) std::cout << std::setw(10) << x << " ";
        std::cout << "\n";
    }
}

int main() {
    Matrix A = {{0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0},
                {-2.0, -3.0, -4.0}};
    Matrix B = {{0.0}, {0.0}, {1.0}};
    Matrix C = {{1.0, 0.0, 0.0}};

    int n = static_cast<int>(A.size());

    Matrix Ctrb = controllabilityMatrix(A, B);
    Matrix Obsv = observabilityMatrix(A, C);

    Matrix CtrbDual = controllabilityMatrix(transpose(A), transpose(C));
    Matrix ObsvDual = observabilityMatrix(transpose(A), transpose(B));

    printMatrix("Ctrb(A,B)", Ctrb);
    printMatrix("Obsv(A,C)", Obsv);

    std::cout << "\nrank Ctrb(A,B) = " << rank(Ctrb) << " of " << n << "\n";
    std::cout << "rank Obsv(A,C) = " << rank(Obsv) << " of " << n << "\n";
    std::cout << "rank Ctrb(A^T,C^T) = " << rank(CtrbDual) << " of " << n << "\n";
    std::cout << "rank Obsv(A^T,B^T) = " << rank(ObsvDual) << " of " << n << "\n";

    return 0;
}
