/*
Chapter16_Lesson3.cpp

Properties of Controllable Canonical Form (CCF) for analysis and design.

This implementation uses only the C++ standard library and implements:
1. CCF matrix construction,
2. controllability matrix construction,
3. determinant/rank by Gaussian elimination,
4. CCF pole-placement gain by coefficient matching.

For production numerical control software, use libraries such as Eigen,
Armadillo, SLICOT, or MATLAB-generated C/C++ code.
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

Matrix zeros(int rows, int cols) {
    return Matrix(rows, Vector(cols, 0.0));
}

void print_matrix(const Matrix& M, const std::string& name) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double v : row) std::cout << std::setw(12) << v << " ";
        std::cout << "\n";
    }
}

void print_vector(const Vector& v, const std::string& name) {
    std::cout << name << " = [ ";
    for (double x : v) std::cout << x << " ";
    std::cout << "]\n";
}

Vector mat_vec(const Matrix& A, const Vector& x) {
    int n = static_cast<int>(A.size());
    int m = static_cast<int>(x.size());
    Vector y(n, 0.0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            y[i] += A[i][j] * x[j];
    return y;
}

Matrix build_A_ccf(const Vector& a) {
    int n = static_cast<int>(a.size());
    Matrix A = zeros(n, n);
    for (int i = 0; i < n - 1; ++i) A[i][i + 1] = 1.0;
    for (int j = 0; j < n; ++j) A[n - 1][j] = -a[j];
    return A;
}

Vector build_B_ccf(int n) {
    Vector B(n, 0.0);
    B[n - 1] = 1.0;
    return B;
}

Matrix controllability_matrix(const Matrix& A, const Vector& B) {
    int n = static_cast<int>(A.size());
    Matrix W = zeros(n, n);
    Vector v = B;
    for (int col = 0; col < n; ++col) {
        for (int row = 0; row < n; ++row) W[row][col] = v[row];
        v = mat_vec(A, v);
    }
    return W;
}

double determinant(Matrix M) {
    int n = static_cast<int>(M.size());
    double det = 1.0;
    int sign = 1;
    const double eps = 1e-12;

    for (int k = 0; k < n; ++k) {
        int pivot = k;
        for (int i = k + 1; i < n; ++i)
            if (std::fabs(M[i][k]) > std::fabs(M[pivot][k])) pivot = i;

        if (std::fabs(M[pivot][k]) < eps) return 0.0;
        if (pivot != k) {
            std::swap(M[pivot], M[k]);
            sign *= -1;
        }

        double p = M[k][k];
        det *= p;
        for (int i = k + 1; i < n; ++i) {
            double factor = M[i][k] / p;
            for (int j = k; j < n; ++j) M[i][j] -= factor * M[k][j];
        }
    }
    return sign * det;
}

int rank_matrix(Matrix M) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int rank = 0;
    const double eps = 1e-10;

    for (int col = 0; col < cols && rank < rows; ++col) {
        int pivot = rank;
        for (int i = rank + 1; i < rows; ++i)
            if (std::fabs(M[i][col]) > std::fabs(M[pivot][col])) pivot = i;

        if (std::fabs(M[pivot][col]) < eps) continue;
        std::swap(M[pivot], M[rank]);

        double p = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= p;

        for (int i = 0; i < rows; ++i) {
            if (i == rank) continue;
            double factor = M[i][col];
            for (int j = col; j < cols; ++j) M[i][j] -= factor * M[rank][j];
        }
        ++rank;
    }
    return rank;
}

Vector pole_placement_gain_ccf(const Vector& a, const Vector& alpha) {
    if (a.size() != alpha.size()) throw std::runtime_error("Size mismatch.");
    Vector K(a.size());
    for (size_t i = 0; i < a.size(); ++i) K[i] = alpha[i] - a[i];
    return K;
}

int main() {
    // D(s) = s^4 + 6 s^3 + 11 s^2 + 6 s + 2
    // a = [a0, a1, a2, a3]
    Vector a = {2.0, 6.0, 11.0, 6.0};

    Matrix A = build_A_ccf(a);
    Vector B = build_B_ccf(static_cast<int>(a.size()));
    Matrix Wc = controllability_matrix(A, B);

    print_matrix(A, "A");
    print_vector(B, "B");
    print_matrix(Wc, "Wc");

    std::cout << "rank(Wc) = " << rank_matrix(Wc) << "\n";
    std::cout << "det(Wc)  = " << determinant(Wc) << "\n";

    // Desired polynomial from roots {-2,-3,-4,-5}:
    // D_des(s) = s^4 + 14s^3 + 71s^2 + 154s + 120.
    Vector alpha = {120.0, 154.0, 71.0, 14.0};
    Vector K = pole_placement_gain_ccf(a, alpha);
    print_vector(alpha, "desired alpha = [alpha0, alpha1, alpha2, alpha3]");
    print_vector(K, "K for u = -Kx + r");

    return 0;
}
