// Chapter12_Lesson5.cpp
// Gramian and Hankel-singular-value preview without external libraries.
// Compile: g++ -std=c++17 Chapter12_Lesson5.cpp -o Chapter12_Lesson5

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec4 = std::array<double, 4>;
using Mat4 = std::array<std::array<double, 4>, 4>;

Mat2 add(const Mat2& X, const Mat2& Y) {
    Mat2 Z{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Z[i][j] = X[i][j] + Y[i][j];
    return Z;
}

Mat2 mul(const Mat2& X, const Mat2& Y) {
    Mat2 Z{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                Z[i][j] += X[i][k] * Y[k][j];
    return Z;
}

Mat2 transpose(const Mat2& X) {
    return {{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}}};
}

void printMat(const char* name, const Mat2& X) {
    std::cout << name << "\n";
    for (int i = 0; i < 2; ++i) {
        std::cout << "  ";
        for (int j = 0; j < 2; ++j)
            std::cout << std::setw(12) << X[i][j] << " ";
        std::cout << "\n";
    }
}

Vec4 solve4(Mat4 A, Vec4 b) {
    for (int k = 0; k < 4; ++k) {
        int pivot = k;
        for (int i = k + 1; i < 4; ++i)
            if (std::abs(A[i][k]) > std::abs(A[pivot][k])) pivot = i;
        if (std::abs(A[pivot][k]) < 1e-12) throw std::runtime_error("Singular system");
        std::swap(A[k], A[pivot]);
        std::swap(b[k], b[pivot]);

        double diag = A[k][k];
        for (int j = k; j < 4; ++j) A[k][j] /= diag;
        b[k] /= diag;

        for (int i = 0; i < 4; ++i) {
            if (i == k) continue;
            double factor = A[i][k];
            for (int j = k; j < 4; ++j) A[i][j] -= factor * A[k][j];
            b[i] -= factor * b[k];
        }
    }
    return b;
}

Mat2 basis(int index) {
    Mat2 E{{{0.0, 0.0}, {0.0, 0.0}}};
    E[index / 2][index % 2] = 1.0;
    return E;
}

Vec4 flatten(const Mat2& X) {
    return {X[0][0], X[0][1], X[1][0], X[1][1]};
}

// Solve A W + W A^T + Q = 0.
Mat2 lyapunovSolve(const Mat2& A, const Mat2& Q) {
    Mat4 K{};
    for (int col = 0; col < 4; ++col) {
        Mat2 E = basis(col);
        Mat2 L = add(mul(A, E), mul(E, transpose(A)));
        Vec4 f = flatten(L);
        for (int row = 0; row < 4; ++row) K[row][col] = f[row];
    }

    Vec4 rhs = flatten(Q);
    for (double& v : rhs) v = -v;
    Vec4 sol = solve4(K, rhs);

    return {{{sol[0], sol[1]}, {sol[2], sol[3]}}};
}

std::array<double, 2> eigenvalues2(const Mat2& X) {
    double tr = X[0][0] + X[1][1];
    double det = X[0][0] * X[1][1] - X[0][1] * X[1][0];
    double disc = std::max(0.0, tr * tr - 4.0 * det);
    double r1 = 0.5 * (tr + std::sqrt(disc));
    double r2 = 0.5 * (tr - std::sqrt(disc));
    return (r1 >= r2) ? std::array<double,2>{r1, r2} : std::array<double,2>{r2, r1};
}

int main() {
    Mat2 A{{{-1.0, 0.3}, {0.0, -2.0}}};

    // B = [1.0; 0.5], so B B^T:
    Mat2 BBt{{{1.0, 0.5}, {0.5, 0.25}}};

    // C = [1.0, -0.2], so C^T C:
    Mat2 CtC{{{1.0, -0.2}, {-0.2, 0.04}}};

    Mat2 Wc = lyapunovSolve(A, BBt);
    Mat2 Wo = lyapunovSolve(transpose(A), CtC);
    Mat2 P = mul(Wc, Wo);
    auto lambda = eigenvalues2(P);

    std::cout << std::fixed << std::setprecision(6);
    printMat("Controllability Gramian Wc:", Wc);
    printMat("\nOutput-energy Gramian Wo:", Wo);
    printMat("\nProduct Wc*Wo:", P);

    std::cout << "\nHankel singular values:\n";
    std::cout << "  sigma1 = " << std::sqrt(std::max(0.0, lambda[0])) << "\n";
    std::cout << "  sigma2 = " << std::sqrt(std::max(0.0, lambda[1])) << "\n";

    std::cout << "\nInterpretation: large sigma means a state direction is both reachable "
              << "with small input energy and visible at the output.\n";
    return 0;
}
