/*
Chapter18_Lesson4.cpp
Repeated eigenvalues and non-diagonalizable systems.

Self-contained computation for a 3 x 3 Jordan block
J = lambda I + N:
exp(J t) = exp(lambda t) [[1, t, t^2/2],
                          [0, 1, t    ],
                          [0, 0, 1    ]].
Compile:
    g++ -std=c++17 -O2 Chapter18_Lesson4.cpp -o Chapter18_Lesson4
*/

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

Mat3 expJordan3(double lambda, double t) {
    const double e = std::exp(lambda * t);
    return {{
        {{e, e * t, e * t * t / 2.0}},
        {{0.0, e, e * t}},
        {{0.0, 0.0, e}}
    }};
}

Vec3 multiply(const Mat3& a, const Vec3& x) {
    Vec3 y{{0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            y[i] += a[i][j] * x[j];
        }
    }
    return y;
}

int main() {
    const double lambda = -0.4;
    const Vec3 x0{{1.0, -2.0, 1.5}};

    std::cout << "Chapter 18 Lesson 4: Jordan response for a defective repeated eigenvalue\n";
    std::cout << "A = [[lambda,1,0],[0,lambda,1],[0,0,lambda]], lambda = "
              << lambda << "\n\n";

    std::cout << std::setw(6) << "t"
              << std::setw(14) << "x1(t)"
              << std::setw(14) << "x2(t)"
              << std::setw(14) << "x3(t)" << "\n";

    for (int k = 0; k <= 8; ++k) {
        const double t = static_cast<double>(k);
        Mat3 phi = expJordan3(lambda, t);
        Vec3 x = multiply(phi, x0);

        std::cout << std::fixed << std::setprecision(6)
                  << std::setw(6) << t
                  << std::setw(14) << x[0]
                  << std::setw(14) << x[1]
                  << std::setw(14) << x[2] << "\n";
    }

    return 0;
}
