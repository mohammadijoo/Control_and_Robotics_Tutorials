/*
Chapter16_Lesson4.cpp

Construct example continuous-time SISO systems in controllable canonical form
and verify the rank of the controllability matrix.

Build:
    g++ -std=c++17 Chapter16_Lesson4.cpp -O2 -o Chapter16_Lesson4
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

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

double determinant(Matrix A) {
    const int n = static_cast<int>(A.size());
    double det = 1.0;

    for (int i = 0; i < n; ++i) {
        int pivot = i;
        for (int r = i + 1; r < n; ++r) {
            if (std::fabs(A[r][i]) > std::fabs(A[pivot][i])) pivot = r;
        }
        if (std::fabs(A[pivot][i]) < 1e-12) return 0.0;
        if (pivot != i) {
            std::swap(A[pivot], A[i]);
            det = -det;
        }
        det *= A[i][i];
        for (int r = i + 1; r < n; ++r) {
            double factor = A[r][i] / A[i][i];
            for (int c = i; c < n; ++c) {
                A[r][c] -= factor * A[i][c];
            }
        }
    }
    return det;
}

int rank(Matrix A) {
    const int rows = static_cast<int>(A.size());
    const int cols = static_cast<int>(A[0].size());
    int r = 0;

    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i) {
            if (std::fabs(A[i][c]) > std::fabs(A[pivot][c])) pivot = i;
        }
        if (std::fabs(A[pivot][c]) < 1e-10) continue;
        std::swap(A[pivot], A[r]);

        double piv = A[r][c];
        for (int j = c; j < cols; ++j) A[r][j] /= piv;

        for (int i = 0; i < rows; ++i) {
            if (i == r) continue;
            double factor = A[i][c];
            for (int j = c; j < cols; ++j) A[i][j] -= factor * A[r][j];
        }
        ++r;
    }
    return r;
}

struct StateSpace {
    Matrix A;
    Matrix B;
    Matrix C;
    Matrix D;
};

StateSpace ccf(const Vector& den_desc, const Vector& num_desc) {
    if (den_desc.empty() || std::fabs(den_desc[0]) < 1e-14) {
        throw std::invalid_argument("Leading denominator coefficient must be nonzero.");
    }
    const int n = static_cast<int>(den_desc.size()) - 1;
    if (static_cast<int>(num_desc.size()) > n) {
        throw std::invalid_argument("Only strictly proper systems are handled.");
    }

    Vector den(den_desc.size());
    Vector num(num_desc.size());
    for (std::size_t i = 0; i < den.size(); ++i) den[i] = den_desc[i] / den_desc[0];
    for (std::size_t i = 0; i < num.size(); ++i) num[i] = num_desc[i] / den_desc[0];

    Matrix A = zeros(n, n);
    for (int i = 0; i < n - 1; ++i) A[i][i + 1] = 1.0;
    for (int j = 0; j < n; ++j) {
        A[n - 1][j] = -den[n - j];
    }

    Matrix B = zeros(n, 1);
    B[n - 1][0] = 1.0;

    Matrix C = zeros(1, n);
    int offset = n - static_cast<int>(num.size());
    for (int k = 0; k < static_cast<int>(num.size()); ++k) {
        int descending_index = offset + k;
        int ascending_index = n - 1 - descending_index;
        C[0][ascending_index] = num[k];
    }

    Matrix D = zeros(1, 1);
    return {A, B, C, D};
}

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    const int n = static_cast<int>(A.size());
    Matrix W = zeros(n, n);
    Matrix Ak = identity(n);

    for (int k = 0; k < n; ++k) {
        Matrix col = multiply(Ak, B);
        for (int i = 0; i < n; ++i) W[i][k] = col[i][0];
        Ak = multiply(A, Ak);
    }
    return W;
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double x : row) std::cout << std::setw(12) << std::setprecision(6) << x << " ";
        std::cout << "\n";
    }
}

void runExample(const std::string& name, const Vector& den, const Vector& num) {
    StateSpace sys = ccf(den, num);
    Matrix W = controllabilityMatrix(sys.A, sys.B);

    std::cout << "\n" << std::string(72, '=') << "\n";
    std::cout << name << "\n";
    printMatrix("A", sys.A);
    printMatrix("B", sys.B);
    printMatrix("C", sys.C);
    printMatrix("Wc", W);
    std::cout << "rank(Wc) = " << rank(W) << "\n";
    std::cout << "det(Wc)  = " << determinant(W) << "\n";
}

int main() {
    runExample("Example 1: G1(s) = 2/(s^2 + 3s + 2)", {1, 3, 2}, {2});
    runExample("Example 2: G2(s) = (s + 2)/(s^3 + 6s^2 + 11s + 6)", {1, 6, 11, 6}, {1, 2});
    runExample("Example 3: G3(s) = (0.5s^2 + 1.5s + 1)/(s^3 + 4s^2 + 5s + 2)", {1, 4, 5, 2}, {0.5, 1.5, 1.0});
    return 0;
}
