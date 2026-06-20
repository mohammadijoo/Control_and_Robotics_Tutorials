// Chapter18_Lesson1.cpp
// Scratch verification of a Jordan chain for a 4x4 state matrix.
// Compile:
//   g++ -std=c++17 Chapter18_Lesson1.cpp -o Chapter18_Lesson1
// Run:
//   ./Chapter18_Lesson1

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using Vector = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

Vector matVec(const Matrix& A, const Vector& x) {
    const int n = static_cast<int>(A.size());
    Vector y(n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < static_cast<int>(x.size()); ++j) {
            y[i] += A[i][j] * x[j];
        }
    }
    return y;
}

Matrix subtractLambdaI(Matrix A, double lambda) {
    for (int i = 0; i < static_cast<int>(A.size()); ++i) {
        A[i][i] -= lambda;
    }
    return A;
}

double norm2(const Vector& x) {
    double s = 0.0;
    for (double xi : x) {
        s += xi * xi;
    }
    return std::sqrt(s);
}

Vector subtract(const Vector& a, const Vector& b) {
    Vector c(a.size(), 0.0);
    for (int i = 0; i < static_cast<int>(a.size()); ++i) {
        c[i] = a[i] - b[i];
    }
    return c;
}

void printVector(const std::string& name, const Vector& x) {
    std::cout << name << " = [";
    for (int i = 0; i < static_cast<int>(x.size()); ++i) {
        std::cout << std::setw(8) << x[i];
        if (i + 1 < static_cast<int>(x.size())) {
            std::cout << ", ";
        }
    }
    std::cout << "]^T\n";
}

int main() {
    Matrix A = {
        {2, 1, 0, 0},
        {0, 2, 1, 0},
        {0, 0, 2, 0},
        {0, 0, 0, -1}
    };

    const double lambda = 2.0;
    Matrix N = subtractLambdaI(A, lambda);

    // Choose the highest vector in the length-3 chain.
    Vector v3 = {0, 0, 1, 0};
    Vector v2 = matVec(N, v3);
    Vector v1 = matVec(N, v2);

    std::cout << "Jordan chain for lambda = 2\n";
    printVector("v1", v1);
    printVector("v2", v2);
    printVector("v3", v3);

    Vector Nv1 = matVec(N, v1);
    Vector Nv2 = matVec(N, v2);
    Vector Nv3 = matVec(N, v3);

    std::cout << "\nVerification:\n";
    printVector("N*v1", Nv1);
    printVector("N*v2", Nv2);
    printVector("N*v3", Nv3);

    std::cout << "\nResidual norms:\n";
    std::cout << "||N*v1||_2       = " << norm2(Nv1) << "\n";
    std::cout << "||N*v2 - v1||_2  = " << norm2(subtract(Nv2, v1)) << "\n";
    std::cout << "||N*v3 - v2||_2  = " << norm2(subtract(Nv3, v2)) << "\n";

    std::cout << "\nInterpretation: columns [v1 v2 v3] generate one Jordan block of size 3.\n";
    return 0;
}
