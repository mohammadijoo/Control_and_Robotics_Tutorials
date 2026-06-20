/*
Chapter22_Lesson3.cpp
State Feedback vs Output Feedback (Concept Only)

A minimal 2x2 implementation without external libraries.
Compile: g++ Chapter22_Lesson3.cpp -std=c++17 -O2 -o Chapter22_Lesson3
*/

#include <cmath>
#include <complex>
#include <iostream>
#include <string>

struct Matrix2 {
    double a11, a12, a21, a22;
};

std::pair<std::complex<double>, std::complex<double>> eigenvalues2x2(const Matrix2& M) {
    double tr = M.a11 + M.a22;
    double det = M.a11 * M.a22 - M.a12 * M.a21;
    std::complex<double> disc = std::complex<double>(tr * tr - 4.0 * det, 0.0);
    std::complex<double> root = std::sqrt(disc);
    return {(tr + root) / 2.0, (tr - root) / 2.0};
}

void printSummary(const Matrix2& M, const std::string& name) {
    auto [l1, l2] = eigenvalues2x2(M);
    std::cout << name << " = [[" << M.a11 << ", " << M.a12 << "], ["
              << M.a21 << ", " << M.a22 << "]]\n";
    std::cout << "eigenvalues(" << name << ") = " << l1 << ", " << l2 << "\n";
    std::cout << "stable? " << ((l1.real() < 0.0 && l2.real() < 0.0) ? "true" : "false") << "\n\n";
}

int main() {
    // Plant: x_dot = A x + B u, y = C x
    Matrix2 A{0.0, 1.0, -2.0, -0.4};

    // B = [0; 1]. State feedback K = [k1 k2].
    double k1 = 4.0;
    double k2 = 2.6;
    Matrix2 A_state{A.a11, A.a12, A.a21 - k1, A.a22 - k2};

    // Static output feedback with y = position = [1 0] x and F = [f].
    // Effective gain F*C = [f 0], so velocity feedback is impossible here.
    double f = 4.0;
    Matrix2 A_output_position{A.a11, A.a12, A.a21 - f, A.a22};

    // Full output y = x makes static output feedback equivalent to state feedback.
    Matrix2 A_output_full = A_state;

    printSummary(A, "A_open_loop");
    printSummary(A_state, "A_state_feedback");
    printSummary(A_output_position, "A_static_output_feedback_position_only");
    printSummary(A_output_full, "A_static_output_feedback_full_output");

    std::cout << "Geometry note:\n";
    std::cout << "For C = [1 0], every static output gain has K_eff = F*C = [f 0].\n";
    std::cout << "Therefore K = [4 2.6] is not implementable from position alone.\n";

    return 0;
}
