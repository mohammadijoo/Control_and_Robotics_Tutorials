// Chapter15_Lesson3.cpp
// Implicit Euler for a stiff linear 2x2 system x' = A x
// Build example:
//   g++ -O2 -std=c++17 Chapter15_Lesson3.cpp -o stiff_demo
#include <iostream>
#include <array>
#include <iomanip>
#include <stdexcept>
#include <cmath>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double,2>,2>;

// Solve 2x2 linear system M z = b
Vec2 solve2x2(const Mat2& M, const Vec2& b) {
    double a = M[0][0], c = M[0][1];
    double d = M[1][0], e = M[1][1];
    double det = a*e - c*d;
    if (std::abs(det) < 1e-15) {
        throw std::runtime_error("Singular matrix in solve2x2");
    }
    return {
        ( e*b[0] - c*b[1]) / det,
        (-d*b[0] + a*b[1]) / det
    };
}

int main() {
    // Stiff eigenvalues approximately -1 and -1000 (upper-triangular => diagonal entries are eigenvalues)
    Mat2 A = {{{-1000.0, 999.0},
               {   0.0,   -1.0}}};

    double h = 0.02;
    int N = 50;
    Vec2 x{1.0, 1.0};

    // Implicit Euler: (I - h A) x_{n+1} = x_n
    Mat2 M = {{{1.0 - h*A[0][0], -h*A[0][1]},
               {-h*A[1][0],      1.0 - h*A[1][1]}}};

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Implicit Euler for stiff linear system\n";
    std::cout << "h = " << h << ", steps = " << N << "\n";
    double t = 0.0;
    for (int n = 0; n < N; ++n) {
        x = solve2x2(M, x);
        t += h;
        std::cout << "n=" << std::setw(3) << (n+1)
                  << "  t=" << std::setw(8) << t
                  << "  x=[" << std::setw(12) << x[0]
                  << ", " << std::setw(12) << x[1] << "]\n";
    }

    // For production-grade stiff solvers in C++, see SUNDIALS (CVODE).
    return 0;
}
