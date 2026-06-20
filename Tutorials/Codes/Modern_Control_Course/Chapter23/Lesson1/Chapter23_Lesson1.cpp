// Chapter23_Lesson1.cpp
// Formulating the SISO pole-placement problem for a second-order companion-form system.
// This implementation uses only the C++ standard library.

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <utility>

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

double determinant2x2(const Matrix2& M) {
    return M.a11 * M.a22 - M.a12 * M.a21;
}

int main() {
    // A = [[0, 1], [-2, -3]], b = [[0], [1]]
    Matrix2 A{0.0, 1.0, -2.0, -3.0};
    double b1 = 0.0;
    double b2 = 1.0;

    // Controllability matrix C = [b, A b]
    // b = [0, 1]^T, A b = [1, -3]^T
    Matrix2 Ctrb{b1, A.a11 * b1 + A.a12 * b2,
                 b2, A.a21 * b1 + A.a22 * b2};

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "det(Ctrb) = " << determinant2x2(Ctrb) << "\n";
    std::cout << "controllable = " << (std::abs(determinant2x2(Ctrb)) > 1e-9 ? "true" : "false") << "\n";

    // Open-loop polynomial: s^2 + 3 s + 2.
    // Desired poles: -4 and -5, so desired polynomial: s^2 + 9 s + 20.
    double a0 = 2.0;
    double a1 = 3.0;
    double alpha0 = 20.0;
    double alpha1 = 9.0;

    // For companion form: K = [alpha0 - a0, alpha1 - a1].
    double k0 = alpha0 - a0;
    double k1 = alpha1 - a1;
    std::cout << "K = [" << k0 << ", " << k1 << "]\n";

    // Acl = A - b K. Since b = [0, 1]^T, only the second row changes.
    Matrix2 Acl{A.a11 - b1 * k0, A.a12 - b1 * k1,
                A.a21 - b2 * k0, A.a22 - b2 * k1};

    auto eigs = eigenvalues2x2(Acl);
    std::cout << "A_cl = [[" << Acl.a11 << ", " << Acl.a12 << "], ["
              << Acl.a21 << ", " << Acl.a22 << "]]\n";
    std::cout << "closed-loop eigenvalue 1 = " << eigs.first << "\n";
    std::cout << "closed-loop eigenvalue 2 = " << eigs.second << "\n";

    // Characteristic polynomial of 2x2 matrix: s^2 - tr(Acl)s + det(Acl).
    double p1 = -(Acl.a11 + Acl.a22);
    double p0 = determinant2x2(Acl);
    std::cout << "closed-loop polynomial: s^2 + " << p1 << " s + " << p0 << "\n";

    return 0;
}