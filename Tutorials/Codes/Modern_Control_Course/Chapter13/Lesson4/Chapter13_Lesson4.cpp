// Chapter13_Lesson4.cpp
// Relationship of Observability to Sensor Placement
// Scratch implementation: matrix powers, observability matrix, and rank by Gaussian elimination.
// Compile: g++ -std=c++17 Chapter13_Lesson4.cpp -O2 -o Chapter13_Lesson4

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix eye(int n) {
    Matrix I(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int m = static_cast<int>(A.size());
    int p = static_cast<int>(A[0].size());
    int n = static_cast<int>(B[0].size());
    Matrix C(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; ++i) {
        for (int k = 0; k < p; ++k) {
            for (int j = 0; j < n; ++j) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

Matrix stackVertical(const std::vector<Matrix>& blocks) {
    int cols = static_cast<int>(blocks[0][0].size());
    Matrix out;
    for (const auto& B : blocks) {
        for (const auto& row : B) {
            if (static_cast<int>(row.size()) != cols) {
                throw std::runtime_error("Inconsistent column count.");
            }
            out.push_back(row);
        }
    }
    return out;
}

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = static_cast<int>(A.size());
    Matrix Ak = eye(n);
    std::vector<Matrix> blocks;
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(C, Ak));
        Ak = multiply(Ak, A);
    }
    return stackVertical(blocks);
}

int rankMatrix(Matrix M, double tol = 1e-9) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int r = 0;
    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i) {
            if (std::fabs(M[i][c]) > std::fabs(M[pivot][c])) pivot = i;
        }
        if (std::fabs(M[pivot][c]) <= tol) continue;
        std::swap(M[r], M[pivot]);
        double piv = M[r][c];
        for (int j = c; j < cols; ++j) M[r][j] /= piv;
        for (int i = 0; i < rows; ++i) {
            if (i == r) continue;
            double factor = M[i][c];
            for (int j = c; j < cols; ++j) M[i][j] -= factor * M[r][j];
        }
        ++r;
    }
    return r;
}

Matrix CFromMask(int mask, int n) {
    Matrix C;
    for (int j = 0; j < n; ++j) {
        if (mask & (1 << j)) {
            std::vector<double> row(n, 0.0);
            row[j] = 1.0;
            C.push_back(row);
        }
    }
    return C;
}

int sensorCount(int mask) {
    int count = 0;
    while (mask) {
        count += mask & 1;
        mask >>= 1;
    }
    return count;
}

int main() {
    Matrix A = {
        {0.0, 1.0, 0.0, 0.0},
        {-2.0, -0.4, 0.8, 0.0},
        {0.0, 0.0, -1.0, 1.0},
        {0.6, 0.0, -3.0, -0.5}
    };

    int n = static_cast<int>(A.size());
    std::cout << "Sensor placement by rank of O = [C; C A; ...; C A^(n-1)]\n";

    for (int mask = 1; mask < (1 << n); ++mask) {
        if (sensorCount(mask) > 2) continue;
        Matrix C = CFromMask(mask, n);
        Matrix O = observabilityMatrix(A, C);
        int r = rankMatrix(O);
        std::cout << "Sensors {";
        bool first = true;
        for (int j = 0; j < n; ++j) {
            if (mask & (1 << j)) {
                if (!first) std::cout << ",";
                std::cout << (j + 1);
                first = false;
            }
        }
        std::cout << "} rank = " << r << "\n";
    }

    return 0;
}
