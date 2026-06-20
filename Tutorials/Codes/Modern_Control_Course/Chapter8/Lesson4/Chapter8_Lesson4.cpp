// Chapter8_Lesson4.cpp
// Computing Phi(t) from a known Jordan form: Phi(t)=P exp(Jt) P^{-1}.
// Compile: g++ -std=c++17 Chapter8_Lesson4.cpp -o Chapter8_Lesson4

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix zeros(int r, int c) {
    return Matrix(r, std::vector<double>(c, 0.0));
}

Matrix identity(int n) {
    Matrix I = zeros(n, n);
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int m = static_cast<int>(A[0].size());
    int c = static_cast<int>(B[0].size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int k = 0; k < m; ++k)
            for (int j = 0; j < c; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix inverse(Matrix A) {
    int n = static_cast<int>(A.size());
    Matrix Aug = zeros(n, 2 * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) Aug[i][j] = A[i][j];
        Aug[i][n + i] = 1.0;
    }

    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int r = col + 1; r < n; ++r)
            if (std::fabs(Aug[r][col]) > std::fabs(Aug[pivot][col])) pivot = r;
        if (std::fabs(Aug[pivot][col]) < 1e-12) throw std::runtime_error("Singular matrix.");
        std::swap(Aug[col], Aug[pivot]);

        double p = Aug[col][col];
        for (int j = 0; j < 2 * n; ++j) Aug[col][j] /= p;

        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            double factor = Aug[r][col];
            for (int j = 0; j < 2 * n; ++j) Aug[r][j] -= factor * Aug[col][j];
        }
    }

    Matrix Inv = zeros(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Inv[i][j] = Aug[i][n + j];
    return Inv;
}

double factorial(int n) {
    double f = 1.0;
    for (int i = 2; i <= n; ++i) f *= i;
    return f;
}

Matrix jordanBlockExp(double lambda, int size, double t) {
    Matrix B = zeros(size, size);
    double e = std::exp(lambda * t);
    for (int i = 0; i < size; ++i) {
        for (int j = i; j < size; ++j) {
            int p = j - i;
            B[i][j] = e * std::pow(t, p) / factorial(p);
        }
    }
    return B;
}

Matrix blockDiag(const std::vector<Matrix>& blocks) {
    int n = 0;
    for (const auto& B : blocks) n += static_cast<int>(B.size());
    Matrix M = zeros(n, n);
    int offset = 0;
    for (const auto& B : blocks) {
        int m = static_cast<int>(B.size());
        for (int i = 0; i < m; ++i)
            for (int j = 0; j < m; ++j)
                M[offset + i][offset + j] = B[i][j];
        offset += m;
    }
    return M;
}

void printMatrix(const Matrix& A, const std::string& name) {
    std::cout << name << " =\n";
    for (const auto& row : A) {
        for (double v : row) std::cout << std::setw(14) << std::setprecision(7) << v << " ";
        std::cout << "\n";
    }
}

int main() {
    Matrix P = {
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 1.0},
        {1.0, 0.0, 1.0}
    };

    Matrix expJ = blockDiag({
        jordanBlockExp(2.0, 2, 0.40),
        jordanBlockExp(-1.0, 1, 0.40)
    });

    Matrix Phi = multiply(multiply(P, expJ), inverse(P));
    printMatrix(Phi, "Phi(0.40)");

    return 0;
}
