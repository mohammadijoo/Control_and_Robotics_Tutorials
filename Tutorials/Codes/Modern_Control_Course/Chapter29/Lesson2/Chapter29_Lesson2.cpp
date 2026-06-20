// Chapter29_Lesson2.cpp
// From-scratch 2x2 implementation for the LTV transition matrix.
// Compile:
//   g++ -std=c++17 -O2 Chapter29_Lesson2.cpp -o Chapter29_Lesson2

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct Mat2 {
    std::array<double, 4> a{};
};

Mat2 zero() {
    return Mat2{{0.0, 0.0, 0.0, 0.0}};
}

Mat2 eye() {
    return Mat2{{1.0, 0.0, 0.0, 1.0}};
}

Mat2 add(const Mat2& X, const Mat2& Y) {
    return Mat2{{X.a[0] + Y.a[0], X.a[1] + Y.a[1],
                 X.a[2] + Y.a[2], X.a[3] + Y.a[3]}};
}

Mat2 sub(const Mat2& X, const Mat2& Y) {
    return Mat2{{X.a[0] - Y.a[0], X.a[1] - Y.a[1],
                 X.a[2] - Y.a[2], X.a[3] - Y.a[3]}};
}

Mat2 scale(double c, const Mat2& X) {
    return Mat2{{c * X.a[0], c * X.a[1], c * X.a[2], c * X.a[3]}};
}

Mat2 mul(const Mat2& X, const Mat2& Y) {
    return Mat2{{
        X.a[0] * Y.a[0] + X.a[1] * Y.a[2],
        X.a[0] * Y.a[1] + X.a[1] * Y.a[3],
        X.a[2] * Y.a[0] + X.a[3] * Y.a[2],
        X.a[2] * Y.a[1] + X.a[3] * Y.a[3]
    }};
}

double normF(const Mat2& X) {
    double s = 0.0;
    for (double v : X.a) s += v * v;
    return std::sqrt(s);
}

Mat2 A(double t) {
    return Mat2{{
        0.0, 1.0,
        -2.0 - 0.5 * std::sin(t), -0.4 + 0.2 * std::cos(2.0 * t)
    }};
}

Mat2 rhs(double t, const Mat2& Phi) {
    return mul(A(t), Phi);
}

Mat2 rk4Phi(double t0, double tf, int steps) {
    double h = (tf - t0) / static_cast<double>(steps);
    double t = t0;
    Mat2 Phi = eye();

    for (int i = 0; i < steps; ++i) {
        Mat2 k1 = rhs(t, Phi);
        Mat2 k2 = rhs(t + 0.5 * h, add(Phi, scale(0.5 * h, k1)));
        Mat2 k3 = rhs(t + 0.5 * h, add(Phi, scale(0.5 * h, k2)));
        Mat2 k4 = rhs(t + h, add(Phi, scale(h, k3)));

        Mat2 incr = scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4)));
        Phi = add(Phi, incr);
        t += h;
    }
    return Phi;
}

Mat2 peanoBaker(double t0, double tf, int order, int steps) {
    double h = (tf - t0) / static_cast<double>(steps);
    std::vector<std::vector<Mat2>> terms(order + 1, std::vector<Mat2>(steps + 1, zero()));

    for (int j = 0; j <= steps; ++j) terms[0][j] = eye();

    for (int k = 1; k <= order; ++k) {
        terms[k][0] = zero();
        for (int j = 1; j <= steps; ++j) {
            double smid = t0 + (j - 0.5) * h;
            Mat2 integrand = mul(A(smid), terms[k - 1][j - 1]);
            terms[k][j] = add(terms[k][j - 1], scale(h, integrand));
        }
    }

    Mat2 Phi = zero();
    for (int k = 0; k <= order; ++k) Phi = add(Phi, terms[k][steps]);
    return Phi;
}

void printMat(const Mat2& X) {
    std::cout << "[" << std::setw(12) << X.a[0] << " " << std::setw(12) << X.a[1] << "]\n";
    std::cout << "[" << std::setw(12) << X.a[2] << " " << std::setw(12) << X.a[3] << "]\n";
}

int main() {
    std::cout << std::setprecision(8) << std::fixed;

    double t0 = 0.0;
    double tf = 6.0;
    Mat2 PhiRK = rk4Phi(t0, tf, 20000);

    std::cout << "Phi(tf,t0) by RK4 matrix IVP:\n";
    printMat(PhiRK);

    for (int order : {1, 2, 3, 4, 6, 8}) {
        Mat2 PhiPB = peanoBaker(t0, tf, order, 6000);
        std::cout << "Peano-Baker order " << order
                  << " Frobenius error = " << normF(sub(PhiPB, PhiRK)) << "\n";
    }

    Mat2 Phi60 = rk4Phi(0.0, 6.0, 20000);
    Mat2 Phi62 = rk4Phi(2.5, 6.0, 12000);
    Mat2 Phi20 = rk4Phi(0.0, 2.5, 8000);
    std::cout << "Composition error = " << normF(sub(Phi60, mul(Phi62, Phi20))) << "\n";
    return 0;
}
