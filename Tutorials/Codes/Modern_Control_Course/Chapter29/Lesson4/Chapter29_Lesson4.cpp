// Chapter29_Lesson4.cpp
// Numerical controllability and observability Gramians for a 2-state LTV system.
// Build example:
//   g++ -std=c++17 -O2 Chapter29_Lesson4.cpp -o Chapter29_Lesson4

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec2 = std::array<double, 2>;

Mat2 zero2() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Mat2 eye2() {
    return {{{1.0, 0.0}, {0.0, 1.0}}};
}

Mat2 A(double t) {
    return {{{0.0, 1.0},
             {-(2.0 + 0.4 * std::sin(t)), -(0.25 + 0.10 * std::cos(2.0 * t))}}};
}

Vec2 B(double t) {
    return {0.0, 1.0 + 0.25 * std::sin(0.5 * t)};
}

Vec2 C(double t) {
    return {1.0, 0.30 * std::cos(t)};
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

Mat2 mul(const Mat2& X, const Mat2& Y) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                Z[i][j] += X[i][k] * Y[k][j];
    return Z;
}

Mat2 trans(const Mat2& X) {
    return {{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}}};
}

double det2(const Mat2& X) {
    return X[0][0] * X[1][1] - X[0][1] * X[1][0];
}

Mat2 inv2(const Mat2& X) {
    double d = det2(X);
    if (std::abs(d) < 1e-14) {
        std::cerr << "Singular 2x2 matrix encountered.\n";
        return zero2();
    }
    return {{{ X[1][1] / d, -X[0][1] / d},
             {-X[1][0] / d,  X[0][0] / d}}};
}

Mat2 outer(const Vec2& u, const Vec2& v) {
    return {{{u[0] * v[0], u[0] * v[1]},
             {u[1] * v[0], u[1] * v[1]}}};
}

Mat2 phiDerivative(double t, const Mat2& Phi) {
    return mul(A(t), Phi);
}

Mat2 rk4StepPhi(const Mat2& Phi, double t, double h) {
    Mat2 k1 = phiDerivative(t, Phi);
    Mat2 k2 = phiDerivative(t + 0.5 * h, add(Phi, scale(k1, 0.5 * h)));
    Mat2 k3 = phiDerivative(t + 0.5 * h, add(Phi, scale(k2, 0.5 * h)));
    Mat2 k4 = phiDerivative(t + h, add(Phi, scale(k3, h)));

    Mat2 sum = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
    return add(Phi, scale(sum, h / 6.0));
}

void printMat(const Mat2& M) {
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "[" << M[0][0] << ", " << M[0][1] << "]\n";
    std::cout << "[" << M[1][0] << ", " << M[1][1] << "]\n";
}

double minEigSym2(const Mat2& M) {
    double a = M[0][0];
    double b = 0.5 * (M[0][1] + M[1][0]);
    double d = M[1][1];
    double tr = a + d;
    double disc = std::sqrt((a - d) * (a - d) + 4.0 * b * b);
    return 0.5 * (tr - disc);
}

int main() {
    const double t0 = 0.0;
    const double tf = 6.0;
    const int nSteps = 4000;
    const double h = (tf - t0) / static_cast<double>(nSteps);

    std::vector<double> times(nSteps + 1);
    std::vector<Mat2> phis(nSteps + 1);

    Mat2 Phi = eye2();
    phis[0] = Phi;
    times[0] = t0;

    for (int k = 0; k < nSteps; ++k) {
        double t = t0 + k * h;
        Phi = rk4StepPhi(Phi, t, h);
        phis[k + 1] = Phi;
        times[k + 1] = t + h;
    }

    Mat2 PhiTfT0 = phis[nSteps];
    Mat2 Wc = zero2();
    Mat2 Wo = zero2();

    for (int k = 0; k <= nSteps; ++k) {
        double weight = (k == 0 || k == nSteps) ? 0.5 : 1.0;
        double s = times[k];

        Mat2 PhiST0 = phis[k];
        Mat2 PhiTfS = mul(PhiTfT0, inv2(PhiST0));

        Vec2 bs = B(s);
        Vec2 cs = C(s);

        Mat2 wcIntegrand = mul(mul(PhiTfS, outer(bs, bs)), trans(PhiTfS));
        Mat2 woIntegrand = mul(mul(trans(PhiST0), outer(cs, cs)), PhiST0);

        Wc = add(Wc, scale(wcIntegrand, weight * h));
        Wo = add(Wo, scale(woIntegrand, weight * h));
    }

    std::cout << "Controllability Gramian Wc:\n";
    printMat(Wc);
    std::cout << "det(Wc): " << det2(Wc) << "\n";
    std::cout << "min eig approx Wc: " << minEigSym2(Wc) << "\n\n";

    std::cout << "Observability Gramian Wo:\n";
    printMat(Wo);
    std::cout << "det(Wo): " << det2(Wo) << "\n";
    std::cout << "min eig approx Wo: " << minEigSym2(Wo) << "\n";

    return 0;
}
