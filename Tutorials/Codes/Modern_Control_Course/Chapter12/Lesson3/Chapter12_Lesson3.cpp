// Chapter12_Lesson3.cpp
// Finite-horizon Gramian test for a 2-state continuous-time LTI system.
// Standard C++17 only: no external linear algebra library.

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec2 = std::array<double, 2>;

Mat2 zero2() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Mat2 identity2() {
    return {{{1.0, 0.0}, {0.0, 1.0}}};
}

Mat2 add(const Mat2& X, const Mat2& Y) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Z[i][j] = X[i][j] + Y[i][j];
    return Z;
}

Mat2 scale(const Mat2& X, double a) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Z[i][j] = a * X[i][j];
    return Z;
}

Mat2 multiply(const Mat2& X, const Mat2& Y) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                Z[i][j] += X[i][k] * Y[k][j];
    return Z;
}

Mat2 transpose(const Mat2& X) {
    return {{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}}};
}

Mat2 outer(const Vec2& b) {
    return {{{b[0] * b[0], b[0] * b[1]},
             {b[1] * b[0], b[1] * b[1]}}};
}

Mat2 expm_taylor(const Mat2& A, double t, int terms = 40) {
    Mat2 At = scale(A, t);
    Mat2 E = identity2();
    Mat2 P = identity2();
    double factorial = 1.0;

    for (int k = 1; k <= terms; ++k) {
        P = multiply(P, At);
        factorial *= static_cast<double>(k);
        E = add(E, scale(P, 1.0 / factorial));
    }
    return E;
}

Mat2 integrand(const Mat2& A, const Vec2& b, double tau) {
    Mat2 E = expm_taylor(A, tau);
    Mat2 BBt = outer(b);
    return multiply(multiply(E, BBt), transpose(E));
}

Mat2 finite_horizon_gramian(const Mat2& A, const Vec2& b, double T, int N = 2000) {
    if (N % 2 == 1) ++N; // Simpson rule requires even number of subintervals.
    double h = T / static_cast<double>(N);
    Mat2 W = zero2();

    for (int k = 0; k <= N; ++k) {
        double tau = k * h;
        double coeff = (k == 0 || k == N) ? 1.0 : ((k % 2 == 0) ? 2.0 : 4.0);
        W = add(W, scale(integrand(A, b, tau), coeff));
    }

    W = scale(W, h / 3.0);
    W[0][1] = W[1][0] = 0.5 * (W[0][1] + W[1][0]);
    return W;
}

double det2(const Mat2& W) {
    return W[0][0] * W[1][1] - W[0][1] * W[1][0];
}

void symmetric_eigenvalues(const Mat2& W, double& lambda_min, double& lambda_max) {
    double a = W[0][0];
    double d = W[1][1];
    double c = 0.5 * (W[0][1] + W[1][0]);
    double tr = a + d;
    double disc = std::sqrt((a - d) * (a - d) + 4.0 * c * c);
    lambda_min = 0.5 * (tr - disc);
    lambda_max = 0.5 * (tr + disc);
}

void print_matrix(const Mat2& W) {
    std::cout << std::fixed << std::setprecision(10);
    std::cout << "[" << W[0][0] << ", " << W[0][1] << "]\n";
    std::cout << "[" << W[1][0] << ", " << W[1][1] << "]\n";
}

void report(const Mat2& W, double tol = 1e-8) {
    double lmin, lmax;
    symmetric_eigenvalues(W, lmin, lmax);
    std::cout << "det(W) = " << det2(W) << "\n";
    std::cout << "lambda_min = " << lmin << "\n";
    std::cout << "lambda_max = " << lmax << "\n";
    std::cout << "positive definite? " << (lmin > tol ? "yes" : "no") << "\n";
    if (lmin > tol)
        std::cout << "condition number = " << lmax / lmin << "\n";
    else
        std::cout << "condition number = infinity\n";
}

int main() {
    Mat2 A1 = {{{0.0, 1.0}, {-2.0, -3.0}}};
    Vec2 b1 = {0.0, 1.0};
    double T = 2.0;

    std::cout << "Controllable example Wc(T):\n";
    Mat2 W1 = finite_horizon_gramian(A1, b1, T);
    print_matrix(W1);
    report(W1);

    Mat2 A2 = {{{0.0, 0.0}, {0.0, -1.0}}};
    Vec2 b2 = {1.0, 0.0};

    std::cout << "\nUncontrollable example Wc(T):\n";
    Mat2 W2 = finite_horizon_gramian(A2, b2, T);
    print_matrix(W2);
    report(W2);

    return 0;
}
