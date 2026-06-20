// Chapter20_Lesson1.cpp
// Minimal realization tests for continuous-time LTI systems.
// Compile: g++ -std=c++17 Chapter20_Lesson1.cpp -O2 -o Chapter20_Lesson1
//
// The transfer matrix is G(s) = C (sI - A)^(-1) B + D.
// A realization is minimal iff (A,B) is reachable and (C,A) is observable.

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
    if (static_cast<int>(B.size()) != k) throw std::runtime_error("Dimension mismatch.");
    Matrix M = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            for (int t = 0; t < k; ++t)
                M[i][j] += A[i][t] * B[t][j];
    return M;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int r = static_cast<int>(blocks[0].size());
    int totalCols = 0;
    for (const auto& B : blocks) totalCols += static_cast<int>(B[0].size());
    Matrix M = zeros(r, totalCols);
    int col0 = 0;
    for (const auto& B : blocks) {
        int c = static_cast<int>(B[0].size());
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < c; ++j)
                M[i][col0 + j] = B[i][j];
        col0 += c;
    }
    return M;
}

Matrix vstack(const std::vector<Matrix>& blocks) {
    int c = static_cast<int>(blocks[0][0].size());
    int totalRows = 0;
    for (const auto& B : blocks) totalRows += static_cast<int>(B.size());
    Matrix M = zeros(totalRows, c);
    int row0 = 0;
    for (const auto& B : blocks) {
        int r = static_cast<int>(B.size());
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < c; ++j)
                M[row0 + i][j] = B[i][j];
        row0 += r;
    }
    return M;
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

Matrix reachabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    std::vector<Matrix> blocks;
    Matrix Ak = identity(n);
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(Ak, B));
        Ak = multiply(A, Ak);
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

bool isReachable(const Matrix& A, const Matrix& B) {
    return rank(reachabilityMatrix(A, B)) == static_cast<int>(A.size());
}

bool isObservable(const Matrix& A, const Matrix& C) {
    return rank(observabilityMatrix(A, C)) == static_cast<int>(A.size());
}

bool isMinimal(const Matrix& A, const Matrix& B, const Matrix& C) {
    return isReachable(A, B) && isObservable(A, C);
}

void report(const std::string& name, const Matrix& A, const Matrix& B, const Matrix& C) {
    std::cout << "\n" << name << "\n";
    std::cout << std::string(name.size(), '-') << "\n";
    std::cout << "rank(R_n) = " << rank(reachabilityMatrix(A, B))
              << " of n = " << A.size() << "\n";
    std::cout << "rank(O_n) = " << rank(observabilityMatrix(A, C))
              << " of n = " << A.size() << "\n";
    std::cout << "minimal?  = " << (isMinimal(A, B, C) ? "yes" : "no") << "\n";
}

int main() {
    Matrix A_nonmin = {{-1.0, 0.0},
                       { 0.0,-2.0}};
    Matrix B_nonmin = {{1.0},
                       {1.0}};
    Matrix C_nonmin = {{0.0, 1.0}};

    Matrix A_min = {{-2.0}};
    Matrix B_min = {{1.0}};
    Matrix C_min = {{1.0}};

    report("Two-state nonminimal realization", A_nonmin, B_nonmin, C_nonmin);
    report("One-state minimal realization", A_min, B_min, C_min);

    std::cout << "\nBoth realizations have external transfer behavior G(s)=1/(s+2),\n";
    std::cout << "but the first realization contains one unobservable internal mode.\n";
    return 0;
}
