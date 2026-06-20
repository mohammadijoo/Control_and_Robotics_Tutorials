// Chapter23_Lesson3.cpp
// Ackermann's formula for SISO pole placement: u = -Kx
// Compile: g++ -std=c++17 Chapter23_Lesson3.cpp -O2 -o Chapter23_Lesson3

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using C = std::complex<double>;
using Matrix = std::vector<std::vector<C>>;

Matrix zeros(int r, int c) {
    return Matrix(r, std::vector<C>(c, C(0.0, 0.0)));
}

Matrix eye(int n) {
    Matrix I = zeros(n, n);
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix add(const Matrix& A, const Matrix& B) {
    int r = (int)A.size(), c = (int)A[0].size();
    Matrix R = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            R[i][j] = A[i][j] + B[i][j];
    return R;
}

Matrix scale(C a, const Matrix& A) {
    int r = (int)A.size(), c = (int)A[0].size();
    Matrix R = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            R[i][j] = a * A[i][j];
    return R;
}

Matrix mul(const Matrix& A, const Matrix& B) {
    int r = (int)A.size(), m = (int)A[0].size(), c = (int)B[0].size();
    Matrix R = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int k = 0; k < m; ++k)
            for (int j = 0; j < c; ++j)
                R[i][j] += A[i][k] * B[k][j];
    return R;
}

Matrix mpow(Matrix A, int p) {
    int n = (int)A.size();
    Matrix R = eye(n);
    while (p > 0) {
        if (p & 1) R = mul(R, A);
        A = mul(A, A);
        p >>= 1;
    }
    return R;
}

Matrix inverse(Matrix A) {
    int n = (int)A.size();
    Matrix Aug = zeros(n, 2 * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) Aug[i][j] = A[i][j];
        Aug[i][n + i] = 1.0;
    }
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int r = col + 1; r < n; ++r) {
            if (std::abs(Aug[r][col]) > std::abs(Aug[pivot][col])) pivot = r;
        }
        if (std::abs(Aug[pivot][col]) < 1e-12) {
            throw std::runtime_error("Matrix is singular or ill-conditioned.");
        }
        std::swap(Aug[pivot], Aug[col]);
        C div = Aug[col][col];
        for (int j = 0; j < 2 * n; ++j) Aug[col][j] /= div;
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            C factor = Aug[r][col];
            for (int j = 0; j < 2 * n; ++j) Aug[r][j] -= factor * Aug[col][j];
        }
    }
    Matrix Inv = zeros(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Inv[i][j] = Aug[i][n + j];
    return Inv;
}

Matrix controllability(const Matrix& A, const Matrix& B) {
    int n = (int)A.size();
    Matrix Ctrb = zeros(n, n);
    Matrix block = B;
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) Ctrb[i][k] = block[i][0];
        block = mul(A, block);
    }
    return Ctrb;
}

std::vector<C> polynomialFromRoots(const std::vector<C>& roots) {
    std::vector<C> coeff = {1.0};
    for (C r : roots) {
        std::vector<C> next(coeff.size() + 1, 0.0);
        for (size_t i = 0; i < coeff.size(); ++i) {
            next[i] += coeff[i];
            next[i + 1] += -r * coeff[i];
        }
        coeff = next;
    }
    return coeff; // [1, alpha_{n-1}, ..., alpha_0]
}

Matrix matrixPolynomial(const Matrix& A, const std::vector<C>& coeff) {
    int n = (int)A.size();
    Matrix R = mpow(A, n);
    for (int i = 1; i <= n; ++i) {
        int power = n - i;
        Matrix term = (power == 0) ? eye(n) : mpow(A, power);
        R = add(R, scale(coeff[i], term));
    }
    return R;
}

Matrix ackermannGain(const Matrix& A, const Matrix& B, const std::vector<C>& desiredPoles) {
    int n = (int)A.size();
    Matrix Ctrb = controllability(A, B);
    Matrix CtrbInv = inverse(Ctrb);
    std::vector<C> p = polynomialFromRoots(desiredPoles);
    Matrix phiA = matrixPolynomial(A, p);
    Matrix eT = zeros(1, n);
    eT[0][n - 1] = 1.0;
    return mul(mul(eT, CtrbInv), phiA);
}

int main() {
    Matrix A = {{0.0, 1.0}, {-2.0, -3.0}};
    Matrix B = {{0.0}, {1.0}};
    std::vector<C> desired = {C(-2.0, 2.0), C(-2.0, -2.0)};

    Matrix K = ackermannGain(A, B, desired);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "K = [ ";
    for (auto v : K[0]) std::cout << v.real() << " ";
    std::cout << "]\n";
    std::cout << "Expected for this example: K = [ 6 1 ]\n";
    return 0;
}
