// Chapter24_Lesson2.cpp
// Degrees of freedom and closed-loop eigenstructure for MIMO pole placement.
// No external linear algebra library is required for this small 3-state example.
//
// Compile:
//   g++ Chapter24_Lesson2.cpp -O2 -std=c++17 -o Chapter24_Lesson2
//
// Run:
//   ./Chapter24_Lesson2

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double x : row) {
            std::cout << std::setw(14) << std::setprecision(8) << x << " ";
        }
        std::cout << "\n";
    }
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    const int r = static_cast<int>(A.size());
    const int c = static_cast<int>(B[0].size());
    const int inner = static_cast<int>(B.size());
    Matrix C(r, std::vector<double>(c, 0.0));
    for (int i = 0; i < r; ++i) {
        for (int k = 0; k < inner; ++k) {
            for (int j = 0; j < c; ++j) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

double det3(const Matrix& M) {
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

Matrix inverse3(const Matrix& M) {
    double d = det3(M);
    if (std::abs(d) < 1e-12) {
        throw std::runtime_error("Singular or nearly singular V matrix.");
    }

    Matrix Inv(3, std::vector<double>(3, 0.0));
    Inv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / d;
    Inv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) / d;
    Inv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / d;

    Inv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) / d;
    Inv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / d;
    Inv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) / d;

    Inv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / d;
    Inv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) / d;
    Inv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / d;

    return Inv;
}

void eigenstructureColumn(double lambda, double gamma,
                          std::vector<double>& v,
                          std::vector<double>& z) {
    // Solve (A - lambda I) v = B z for the example system.
    const double v1 = 1.0;
    const double v2 = lambda;
    const double v3 = lambda * lambda + gamma;
    const double z1 = gamma;
    const double z2 = -2.0 - 3.0 * lambda + (-4.0 - lambda) * v3;

    v = {v1, v2, v3};
    z = {z1, z2};
}

int main() {
    Matrix A = {{0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0},
                {-2.0, -3.0, -4.0}};

    Matrix B = {{0.0, 0.0},
                {1.0, 0.0},
                {0.0, 1.0}};

    std::vector<double> lambdas = {-1.0, -2.0, -5.0};
    std::vector<double> gammas  = {0.2, -0.4, 0.8};

    Matrix V(3, std::vector<double>(3, 0.0));
    Matrix Z(2, std::vector<double>(3, 0.0));

    for (int j = 0; j < 3; ++j) {
        std::vector<double> v, z;
        eigenstructureColumn(lambdas[j], gammas[j], v, z);
        for (int i = 0; i < 3; ++i) V[i][j] = v[i];
        for (int i = 0; i < 2; ++i) Z[i][j] = z[i];
    }

    Matrix K = multiply(Z, inverse3(V));

    printMatrix("A", A);
    printMatrix("B", B);
    printMatrix("V", V);
    printMatrix("Z = K V", Z);
    printMatrix("K", K);

    std::cout << "\nThe desired eigenvalues are: ";
    for (double x : lambdas) std::cout << x << " ";
    std::cout << "\n";
    std::cout << "For this construction, A V - B Z = V Lambda by design.\n";
    std::cout << "Closed-loop matrix is A - B K.\n";

    return 0;
}
