/*
Chapter10_Lesson4.cpp

Physical Interpretation: Actuator Placement and Authority
Modern Control - Chapter 10, Lesson 4

This from-scratch C++ example computes the reachability matrix
[B, AB, A^2B, ..., A^(n-1)B] and its numerical rank for several actuator
placements in a two-degree-of-freedom mechanical system.

Build:
    g++ -std=c++17 Chapter10_Lesson4.cpp -O2 -o Chapter10_Lesson4
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
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
    int kdim = static_cast<int>(B.size());
    int c = static_cast<int>(B[0].size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i) {
        for (int k = 0; k < kdim; ++k) {
            for (int j = 0; j < c; ++j) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int rows = static_cast<int>(blocks[0].size());
    int totalCols = 0;
    for (const auto& B : blocks) totalCols += static_cast<int>(B[0].size());

    Matrix H = zeros(rows, totalCols);
    int col0 = 0;
    for (const auto& B : blocks) {
        int cols = static_cast<int>(B[0].size());
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                H[i][col0 + j] = B[i][j];
            }
        }
        col0 += cols;
    }
    return H;
}

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    Matrix Ak = identity(n);
    std::vector<Matrix> blocks;

    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(Ak, B));
        Ak = multiply(Ak, A);
    }
    return hstack(blocks);
}

int numericalRank(Matrix M, double tol = 1e-9) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int rank = 0;

    for (int col = 0; col < cols && rank < rows; ++col) {
        int pivot = rank;
        for (int i = rank + 1; i < rows; ++i) {
            if (std::fabs(M[i][col]) > std::fabs(M[pivot][col])) pivot = i;
        }

        if (std::fabs(M[pivot][col]) <= tol) continue;
        std::swap(M[pivot], M[rank]);

        double piv = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= piv;

        for (int i = 0; i < rows; ++i) {
            if (i == rank) continue;
            double factor = M[i][col];
            for (int j = col; j < cols; ++j) {
                M[i][j] -= factor * M[rank][j];
            }
        }
        ++rank;
    }
    return rank;
}

double columnEnergyScore(const Matrix& C) {
    double sum = 0.0;
    for (const auto& row : C) {
        for (double value : row) sum += value * value;
    }
    return std::sqrt(sum);
}

void analyzePlacement(const std::string& name, const Matrix& A, const Matrix& B) {
    Matrix C = controllabilityMatrix(A, B);
    int rank = numericalRank(C);
    double score = columnEnergyScore(C);

    std::cout << "\nActuator placement: " << name << "\n";
    std::cout << "Rank of [B, AB, ..., A^(n-1)B]: " << rank << " out of "
              << A.size() << "\n";
    std::cout << "Frobenius norm authority score of reachability matrix: "
              << std::fixed << std::setprecision(6) << score << "\n";
}

int main() {
    // State x = [q1, q2, q1dot, q2dot]^T for two coupled unit masses.
    Matrix A = {
        { 0.0,  0.0, 1.0, 0.0},
        { 0.0,  0.0, 0.0, 1.0},
        {-2.0,  1.0,-0.08,0.0},
        { 1.0, -2.0, 0.0,-0.08}
    };

    // B = [0; 0; G] because M = I in qdd = ... + G u.
    Matrix B_mass1 = {
        {0.0},
        {0.0},
        {1.0},
        {0.0}
    };

    Matrix B_mass2 = {
        {0.0},
        {0.0},
        {0.0},
        {1.0}
    };

    Matrix B_common = {
        {0.0},
        {0.0},
        {1.0},
        {1.0}
    };

    Matrix B_both = {
        {0.0, 0.0},
        {0.0, 0.0},
        {1.0, 0.0},
        {0.0, 1.0}
    };

    analyzePlacement("force on mass 1 only", A, B_mass1);
    analyzePlacement("force on mass 2 only", A, B_mass2);
    analyzePlacement("same force on both masses", A, B_common);
    analyzePlacement("independent forces on both masses", A, B_both);

    return 0;
}
