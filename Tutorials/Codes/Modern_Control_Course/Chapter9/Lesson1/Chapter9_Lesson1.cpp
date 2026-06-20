// Chapter9_Lesson1.cpp
// Modern Control — Chapter 9, Lesson 1
// Stability, Asymptotic Stability, and Instability (State-Space View)
//
// Self-contained 2-by-2 implementation.
// For larger systems, production C++ projects commonly use Eigen, Armadillo,
// or LAPACK wrappers for eigenvalues and matrix exponentials.

#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>

struct Matrix2 {
    double a11, a12, a21, a22;

    double trace() const {
        return a11 + a22;
    }

    double det() const {
        return a11 * a22 - a12 * a21;
    }

    void multiply(const double x[2], double y[2]) const {
        y[0] = a11 * x[0] + a12 * x[1];
        y[1] = a21 * x[0] + a22 * x[1];
    }
};

double norm2(const double x[2]) {
    return std::sqrt(x[0] * x[0] + x[1] * x[1]);
}

std::string classify2x2(const Matrix2& A, double tol = 1e-10) {
    double tr = A.trace();
    double determinant = A.det();
    double discriminant = tr * tr - 4.0 * determinant;

    if (discriminant >= 0.0) {
        double root = std::sqrt(discriminant);
        double lambda1 = 0.5 * (tr + root);
        double lambda2 = 0.5 * (tr - root);

        if (lambda1 < -tol && lambda2 < -tol) return "asymptotically stable";
        if (lambda1 > tol || lambda2 > tol) return "unstable";
        return "borderline: inspect semisimplicity";
    }

    double realPart = 0.5 * tr;
    if (realPart < -tol) return "asymptotically stable";
    if (realPart > tol) return "unstable";
    return "marginal center candidate";
}

void addScaled(const double x[2], const double k[2], double scale, double y[2]) {
    y[0] = x[0] + scale * k[0];
    y[1] = x[1] + scale * k[1];
}

void rk4Step(const Matrix2& A, const double x[2], double h, double xNext[2]) {
    double k1[2], k2[2], k3[2], k4[2], temp[2];

    A.multiply(x, k1);
    addScaled(x, k1, 0.5 * h, temp);
    A.multiply(temp, k2);
    addScaled(x, k2, 0.5 * h, temp);
    A.multiply(temp, k3);
    addScaled(x, k3, h, temp);
    A.multiply(temp, k4);

    xNext[0] = x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0;
    xNext[1] = x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0;
}

void reportSystem(const std::string& name, const Matrix2& A, const double x0[2]) {
    std::cout << "\n=== " << name << " ===\n";
    std::cout << "A = [[" << A.a11 << ", " << A.a12 << "], ["
              << A.a21 << ", " << A.a22 << "]]\n";
    std::cout << "trace = " << A.trace() << ", determinant = " << A.det() << "\n";
    std::cout << "classification = " << classify2x2(A) << "\n";

    double h = 0.01;
    int steps = 1000;
    double x[2] = {x0[0], x0[1]};
    double maxNorm = norm2(x);

    for (int k = 0; k < steps; ++k) {
        double xNext[2];
        rk4Step(A, x, h, xNext);
        x[0] = xNext[0];
        x[1] = xNext[1];
        maxNorm = std::max(maxNorm, norm2(x));
    }

    std::cout << "||x(0)|| = " << norm2(x0) << "\n";
    std::cout << "||x(10)|| approx = " << norm2(x) << "\n";
    std::cout << "max ||x(t)|| approx on [0,10] = " << maxNorm << "\n";
}

int main() {
    double x0[2] = {1.0, -0.5};

    Matrix2 stable{-1.0, 2.0, -3.0, -2.0};
    Matrix2 center{0.0, 1.0, -1.0, 0.0};
    Matrix2 unstable{0.2, 1.0, 0.0, -1.0};

    reportSystem("Stable spiral", stable, x0);
    reportSystem("Marginal center", center, x0);
    reportSystem("Unstable saddle/source component", unstable, x0);

    return 0;
}
