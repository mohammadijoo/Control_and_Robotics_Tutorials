/*
Chapter25_Lesson1.cpp
Uncontrollable Modes and Unassignable Poles

A small from-scratch C++ example for a 2-state SISO system.
Compile:
    g++ -std=c++17 Chapter25_Lesson1.cpp -o Chapter25_Lesson1
Run:
    ./Chapter25_Lesson1
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct Mat2 {
    double a11, a12, a21, a22;
};

struct Vec2 {
    double x1, x2;
};

Vec2 mat_vec(const Mat2& A, const Vec2& x) {
    return {A.a11 * x.x1 + A.a12 * x.x2,
            A.a21 * x.x1 + A.a22 * x.x2};
}

double det2(double m11, double m12, double m21, double m22) {
    return m11 * m22 - m12 * m21;
}

int rank_2_by_2_columns(const Vec2& c1, const Vec2& c2, double tol = 1e-10) {
    double d = det2(c1.x1, c2.x1, c1.x2, c2.x2);
    if (std::abs(d) > tol) return 2;
    if (std::abs(c1.x1) > tol || std::abs(c1.x2) > tol ||
        std::abs(c2.x1) > tol || std::abs(c2.x2) > tol) return 1;
    return 0;
}

void print_eigenvalues_2x2(const Mat2& A) {
    double tr = A.a11 + A.a22;
    double det = A.a11 * A.a22 - A.a12 * A.a21;
    double disc = tr * tr - 4.0 * det;
    if (disc >= 0.0) {
        double s = std::sqrt(disc);
        std::cout << "{" << (tr + s) / 2.0 << ", " << (tr - s) / 2.0 << "}";
    } else {
        double real = tr / 2.0;
        double imag = std::sqrt(-disc) / 2.0;
        std::cout << "{" << real << " + " << imag << "i, "
                  << real << " - " << imag << "i}";
    }
}

int pbh_rank_for_diagonal_example(double lambda, const Mat2& A, const Vec2& B, double tol = 1e-10) {
    // For n=2, PBH matrix is [lambda I - A, B], a 2 x 3 matrix.
    // We compute whether any 2x2 minor is nonzero.
    double c1_1 = lambda - A.a11, c1_2 = -A.a21;
    double c2_1 = -A.a12,         c2_2 = lambda - A.a22;
    double c3_1 = B.x1,           c3_2 = B.x2;

    double m12 = det2(c1_1, c2_1, c1_2, c2_2);
    double m13 = det2(c1_1, c3_1, c1_2, c3_2);
    double m23 = det2(c2_1, c3_1, c2_2, c3_2);
    return (std::abs(m12) > tol || std::abs(m13) > tol || std::abs(m23) > tol) ? 2 : 1;
}

int main() {
    Mat2 A{0.0, 0.0, 0.0, 2.0};
    Vec2 B{1.0, 0.0};

    Vec2 AB = mat_vec(A, B);
    int rankC = rank_2_by_2_columns(B, AB);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "A = [[0, 0], [0, 2]], B = [1, 0]^T\n";
    std::cout << "Controllability matrix C = [B, AB] = [["
              << B.x1 << ", " << AB.x1 << "], ["
              << B.x2 << ", " << AB.x2 << "]]\n";
    std::cout << "rank(C) = " << rankC << " out of n = 2\n\n";

    std::cout << "PBH ranks:\n";
    for (double lambda : {0.0, 2.0}) {
        int r = pbh_rank_for_diagonal_example(lambda, A, B);
        std::cout << "  lambda = " << lambda << ", rank([lambda I - A, B]) = "
                  << r << "/2";
        if (r < 2) std::cout << "  <-- uncontrollable mode";
        std::cout << "\n";
    }

    std::cout << "\nClosed-loop eigenvalues for K = [k1, k2]:\n";
    std::vector<std::pair<double, double>> gains = {{0, 0}, {3, 0}, {8, 100}, {-1, -50}};
    for (auto [k1, k2] : gains) {
        Mat2 Acl{A.a11 - B.x1 * k1, A.a12 - B.x1 * k2,
                 A.a21 - B.x2 * k1, A.a22 - B.x2 * k2};
        std::cout << "  K = [" << k1 << ", " << k2 << "] -> eig(A-BK) = ";
        print_eigenvalues_2x2(Acl);
        std::cout << "\n";
    }

    std::cout << "\nThe eigenvalue 2 remains fixed for every K, so it is unassignable.\n";
    return 0;
}
