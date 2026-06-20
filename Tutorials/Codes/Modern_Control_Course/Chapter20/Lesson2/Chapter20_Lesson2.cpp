/*
Chapter20_Lesson2.cpp

Educational C++ implementation for Chapter 20, Lesson 2.

This file checks reachability/observability ranks and demonstrates exact
minimalization for a diagonal nonminimal SISO example:
    A = diag(-1,-2,-3), B = [1,0,1]^T, C = [1,1,0], D = 0.
The transfer function is G(s)=1/(s+1); modes -2 and -3 are nonminimal.

Compile:
    g++ -std=c++17 -O2 Chapter20_Lesson2.cpp -o Chapter20_Lesson2
Run:
    ./Chapter20_Lesson2
*/

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Complex = std::complex<double>;

Matrix zeros(int r, int c) {
    return Matrix(r, std::vector<double>(c, 0.0));
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int k = static_cast<int>(A[0].size());
    int c = static_cast<int>(B[0].size());
    Matrix M = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            for (int p = 0; p < k; ++p)
                M[i][j] += A[i][p] * B[p][j];
    return M;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int rows = static_cast<int>(blocks[0].size());
    int cols = 0;
    for (const auto& B : blocks) cols += static_cast<int>(B[0].size());
    Matrix M = zeros(rows, cols);
    int offset = 0;
    for (const auto& B : blocks) {
        int c = static_cast<int>(B[0].size());
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < c; ++j)
                M[i][offset + j] = B[i][j];
        offset += c;
    }
    return M;
}

Matrix vstack(const std::vector<Matrix>& blocks) {
    int cols = static_cast<int>(blocks[0][0].size());
    int rows = 0;
    for (const auto& B : blocks) rows += static_cast<int>(B.size());
    Matrix M = zeros(rows, cols);
    int offset = 0;
    for (const auto& B : blocks) {
        int r = static_cast<int>(B.size());
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < cols; ++j)
                M[offset + i][j] = B[i][j];
        offset += r;
    }
    return M;
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
            for (int j = c; j < cols; ++j) M[i][j] -= factor * M[r][j];
        }
        ++r;
    }
    return r;
}

Matrix controllability(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    std::vector<Matrix> blocks;
    Matrix AkB = B;
    blocks.push_back(AkB);
    for (int k = 1; k < n; ++k) {
        AkB = multiply(A, AkB);
        blocks.push_back(AkB);
    }
    return hstack(blocks);
}

Matrix observability(const Matrix& A, const Matrix& C) {
    int n = static_cast<int>(A.size());
    std::vector<Matrix> blocks;
    Matrix CAk = C;
    blocks.push_back(CAk);
    for (int k = 1; k < n; ++k) {
        CAk = multiply(CAk, A);
        blocks.push_back(CAk);
    }
    return vstack(blocks);
}

Complex G_full(Complex s) {
    // For diagonal A, C(sI-A)^(-1)B + D is computed term-by-term.
    Complex term1 = 1.0 / (s + 1.0);      // C1*B1/(s+1)
    Complex term2 = 0.0 / (s + 2.0);      // C2*B2/(s+2), unreachable
    Complex term3 = 0.0 / (s + 3.0);      // C3*B3/(s+3), unobservable
    return term1 + term2 + term3;
}

Complex G_min(Complex s) {
    double Am = -1.0;
    double Bm = 1.0;
    double Cm = 1.0;
    double Dm = 0.0;
    return Cm * (1.0 / (s - Am)) * Bm + Dm;
}

int main() {
    Matrix A = {{-1.0, 0.0, 0.0},
                { 0.0,-2.0, 0.0},
                { 0.0, 0.0,-3.0}};
    Matrix B = {{1.0}, {0.0}, {1.0}};
    Matrix C = {{1.0, 1.0, 0.0}};

    Matrix Wc = controllability(A, B);
    Matrix Wo = observability(A, C);

    std::cout << "rank(Wc) = " << rank(Wc) << " out of n = 3\n";
    std::cout << "rank(Wo) = " << rank(Wo) << " out of n = 3\n\n";

    std::cout << "Original nonminimal realization has modes -1, -2, -3.\n";
    std::cout << "Mode -2 is unreachable; mode -3 is unobservable.\n";
    std::cout << "Minimal realization: Am=[-1], Bm=[1], Cm=[1], Dm=[0].\n\n";

    std::vector<Complex> samples = {0.0, 1.0, Complex(2.0, 1.0)};
    std::cout << std::setprecision(8);
    for (const auto& s : samples) {
        std::cout << "s = " << s
                  << "  G_full(s) = " << G_full(s)
                  << "  G_min(s) = " << G_min(s) << "\n";
    }

    return 0;
}
