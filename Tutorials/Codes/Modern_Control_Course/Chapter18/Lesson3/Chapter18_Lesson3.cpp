// Chapter18_Lesson3.cpp
// Dynamics associated with a size-3 Jordan block.
//
// Compile:
//   g++ -std=c++17 Chapter18_Lesson3.cpp -o Chapter18_Lesson3
//
// Run:
//   ./Chapter18_Lesson3

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;

Matrix3 jordan_exponential(double lambda, double t) {
    // J = lambda I + N, where N has ones on the first superdiagonal.
    // exp(Jt) = exp(lambda t) * [[1, t, t^2/2], [0, 1, t], [0, 0, 1]].
    const double e = std::exp(lambda * t);
    Matrix3 E {{
        {{e, e * t, e * t * t / 2.0}},
        {{0.0, e, e * t}},
        {{0.0, 0.0, e}}
    }};
    return E;
}

Vector3 matvec(const Matrix3& A, const Vector3& x) {
    Vector3 y {{0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            y[i] += A[i][j] * x[j];
        }
    }
    return y;
}

void print_matrix(const Matrix3& A) {
    for (const auto& row : A) {
        for (double value : row) {
            std::cout << std::setw(14) << std::setprecision(7) << value << " ";
        }
        std::cout << "\n";
    }
}

void print_vector(const Vector3& x) {
    std::cout << "[";
    for (int i = 0; i < 3; ++i) {
        std::cout << std::setprecision(7) << x[i];
        if (i < 2) {
            std::cout << ", ";
        }
    }
    std::cout << "]";
}

int main() {
    const double lambda = -0.40;
    const Vector3 z0 {{1.0, -0.5, 2.0}};

    std::cout << "Dynamics for zdot = Jz, J = lambda I + N\n";
    std::cout << "lambda = " << lambda << "\n\n";

    for (double t : {0.0, 1.0, 2.0, 4.0, 8.0}) {
        Matrix3 E = jordan_exponential(lambda, t);
        Vector3 z = matvec(E, z0);

        std::cout << "t = " << t << "\n";
        std::cout << "exp(Jt) =\n";
        print_matrix(E);
        std::cout << "z(t) = ";
        print_vector(z);
        std::cout << "\n\n";
    }

    return 0;
}
