// Chapter24_Lesson3.cpp
// State transformations for MIMO pole placement without external libraries.
// Compile: g++ -std=c++17 Chapter24_Lesson3.cpp -o Chapter24_Lesson3

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix zeros(int r, int c) { return Matrix(r, std::vector<double>(c, 0.0)); }

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = (int)A.size(), n = (int)B.size(), c = (int)B[0].size();
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int k = 0; k < n; ++k)
            for (int j = 0; j < c; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix subtract(const Matrix& A, const Matrix& B) {
    Matrix C = A;
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < A[0].size(); ++j)
            C[i][j] -= B[i][j];
    return C;
}

Matrix inverse(Matrix A) {
    int n = (int)A.size();
    Matrix aug = zeros(n, 2*n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) aug[i][j] = A[i][j];
        aug[i][n+i] = 1.0;
    }
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int r = col + 1; r < n; ++r)
            if (std::fabs(aug[r][col]) > std::fabs(aug[pivot][col])) pivot = r;
        if (std::fabs(aug[pivot][col]) < 1e-12) throw std::runtime_error("Singular matrix");
        std::swap(aug[pivot], aug[col]);
        double div = aug[col][col];
        for (int j = 0; j < 2*n; ++j) aug[col][j] /= div;
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            double factor = aug[r][col];
            for (int j = 0; j < 2*n; ++j) aug[r][j] -= factor * aug[col][j];
        }
    }
    Matrix inv = zeros(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            inv[i][j] = aug[i][n+j];
    return inv;
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << "\n";
    for (const auto& row : M) {
        for (double v : row) std::cout << std::setw(12) << std::setprecision(6) << std::fixed << v << " ";
        std::cout << "\n";
    }
}

double frobeniusNorm(const Matrix& M) {
    double s = 0.0;
    for (const auto& row : M)
        for (double v : row) s += v * v;
    return std::sqrt(s);
}

int main() {
    Matrix Abar = {
        {0,1,0,0},
        {0,0,0,0},
        {0,0,0,1},
        {0,0,0,0}
    };
    Matrix Bbar = {
        {0,0},
        {1,0},
        {0,0},
        {0,1}
    };
    Matrix T = {
        {1.0,0.2,0.0,0.1},
        {0.1,1.0,0.2,0.0},
        {0.0,0.1,1.0,0.3},
        {0.2,0.0,0.1,1.0}
    };
    Matrix F = {
        {6.0,5.0,0.0,0.0},
        {0.0,0.0,20.0,9.0}
    };

    Matrix Tinv = inverse(T);
    Matrix A = multiply(multiply(T, Abar), Tinv);
    Matrix B = multiply(T, Bbar);
    Matrix K = multiply(F, Tinv);
    Matrix Acl = subtract(A, multiply(B, K));
    Matrix Abarcl = subtract(Abar, multiply(Bbar, F));
    Matrix check = subtract(multiply(multiply(Tinv, Acl), T), Abarcl);

    printMatrix("K = F*T^{-1}:", K);
    printMatrix("A - B*K:", Acl);
    std::cout << "Similarity residual Frobenius norm = " << frobeniusNorm(check) << "\n";
    std::cout << "Expected poles from the transformed design: -2, -3, -4, -5\n";
    return 0;
}
