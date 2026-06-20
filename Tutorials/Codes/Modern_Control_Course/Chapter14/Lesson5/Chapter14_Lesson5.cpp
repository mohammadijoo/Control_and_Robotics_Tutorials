// Chapter14_Lesson5.cpp
// Continuous-time LTV observability Gramian using a small RK4 integrator.
// No external linear algebra library is required for this 2-state example.

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Mat2 = std::array<std::array<double, 2>, 2>;

Mat2 zeros() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Mat2 identity() {
    return {{{1.0, 0.0}, {0.0, 1.0}}};
}

Mat2 add(const Mat2& A, const Mat2& B) {
    Mat2 C = zeros();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Mat2 scale(const Mat2& A, double s) {
    Mat2 C = zeros();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            C[i][j] = s * A[i][j];
    return C;
}

Mat2 multiply(const Mat2& A, const Mat2& B) {
    Mat2 C = zeros();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Mat2 transpose(const Mat2& A) {
    return {{{A[0][0], A[1][0]}, {A[0][1], A[1][1]}}};
}

Mat2 A_of_t(double t) {
    return {{{0.0, 1.0},
             {-(2.0 + 0.5 * std::sin(1.3 * t)), -(0.15 + 0.05 * std::cos(t))}}};
}

Mat2 CtC_of_t(double t) {
    double c1 = 1.0;
    double c2 = 0.2 * std::sin(0.7 * t);
    return {{{c1 * c1, c1 * c2}, {c2 * c1, c2 * c2}}};
}

Mat2 phi_rhs(double t, const Mat2& Phi) {
    return multiply(A_of_t(t), Phi);
}

Mat2 rk4_phi_step(double t, const Mat2& Phi, double h) {
    Mat2 k1 = phi_rhs(t, Phi);
    Mat2 k2 = phi_rhs(t + 0.5 * h, add(Phi, scale(k1, 0.5 * h)));
    Mat2 k3 = phi_rhs(t + 0.5 * h, add(Phi, scale(k2, 0.5 * h)));
    Mat2 k4 = phi_rhs(t + h, add(Phi, scale(k3, h)));

    Mat2 incr = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
    return add(Phi, scale(incr, h / 6.0));
}

Mat2 gramian_integrand(double t, const Mat2& Phi) {
    return multiply(multiply(transpose(Phi), CtC_of_t(t)), Phi);
}

void print_matrix(const Mat2& M) {
    std::cout << std::fixed << std::setprecision(8);
    for (int i = 0; i < 2; ++i) {
        std::cout << "[ ";
        for (int j = 0; j < 2; ++j) std::cout << std::setw(12) << M[i][j] << " ";
        std::cout << "]\n";
    }
}

int main() {
    const double t0 = 0.0;
    const double tf = 6.0;
    const int steps = 6000;
    const double h = (tf - t0) / steps;

    Mat2 Phi = identity();
    Mat2 W = zeros();

    double t = t0;
    Mat2 F_prev = gramian_integrand(t, Phi);

    for (int k = 0; k < steps; ++k) {
        Mat2 Phi_next = rk4_phi_step(t, Phi, h);
        double t_next = t + h;
        Mat2 F_next = gramian_integrand(t_next, Phi_next);

        W = add(W, scale(add(F_prev, F_next), 0.5 * h));

        Phi = Phi_next;
        F_prev = F_next;
        t = t_next;
    }

    double detW = W[0][0] * W[1][1] - W[0][1] * W[1][0];

    std::cout << "Approximate observability Gramian W_o:\n";
    print_matrix(W);
    std::cout << "det(W_o) = " << std::setprecision(12) << detW << "\n";
    if (detW > 1e-8) {
        std::cout << "The numerical test indicates observability on [0, 6].\n";
    } else {
        std::cout << "The numerical test indicates loss or near-loss of observability.\n";
    }
    return 0;
}
