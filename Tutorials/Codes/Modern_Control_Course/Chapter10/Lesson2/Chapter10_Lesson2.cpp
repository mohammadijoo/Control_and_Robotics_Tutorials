// Chapter10_Lesson2.cpp
// Reachable States and Reachable Subspace for continuous-time LTI systems
//
// This from-scratch C++ example builds the algebraic reachable subspace matrix
//   R = [B, AB, A^2B, ..., A^(n-1)B]
// and estimates independent columns using Gaussian elimination.
//
// Compile:
//   g++ -std=c++17 Chapter10_Lesson2.cpp -o Chapter10_Lesson2

#include <cmath>
#include <iomanip>
#include <iostream>
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
    int k = static_cast<int>(A[0].size());
    int c = static_cast<int>(B[0].size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            for (int ell = 0; ell < k; ++ell) {
                C[i][j] += A[i][ell] * B[ell][j];
            }
        }
    }
    return C;
}

Matrix hstack(const std::vector<Matrix>& blocks) {
    int rows = static_cast<int>(blocks[0].size());
    int totalCols = 0;
    for (const auto& M : blocks) totalCols += static_cast<int>(M[0].size());

    Matrix H = zeros(rows, totalCols);
    int offset = 0;
    for (const auto& M : blocks) {
        int cols = static_cast<int>(M[0].size());
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                H[i][offset + j] = M[i][j];
        offset += cols;
    }
    return H;
}

Matrix reachabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    Matrix Ak = identity(n);
    std::vector<Matrix> blocks;
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(Ak, B));
        Ak = multiply(Ak, A);
    }
    return hstack(blocks);
}

int rankGaussian(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int rank = 0;

    for (int col = 0; col < cols && rank < rows; ++col) {
        int pivot = rank;
        for (int r = rank + 1; r < rows; ++r) {
            if (std::fabs(M[r][col]) > std::fabs(M[pivot][col])) pivot = r;
        }
        if (std::fabs(M[pivot][col]) <= tol) continue;

        std::swap(M[pivot], M[rank]);
        double pivotValue = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= pivotValue;

        for (int r = 0; r < rows; ++r) {
            if (r == rank) continue;
            double factor = M[r][col];
            for (int j = col; j < cols; ++j) {
                M[r][j] -= factor * M[rank][j];
            }
        }
        ++rank;
    }
    return rank;
}

void printMatrix(const Matrix& M, const std::string& name) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double value : row) {
            std::cout << std::setw(12) << std::setprecision(6) << value << " ";
        }
        std::cout << "\n";
    }
}

int main() {
    Matrix A = {
        {0.0, 1.0, 0.0},
        {-2.0, -3.0, 0.0},
        {0.0, 0.0, -1.0}
    };

    Matrix B = {
        {0.0},
        {1.0},
        {0.0}
    };

    Matrix R = reachabilityMatrix(A, B);

    printMatrix(A, "A");
    printMatrix(B, "B");
    printMatrix(R, "R = [B AB A^2B]");

    int reachableDimension = rankGaussian(R);
    std::cout << "\nEstimated dimension of reachable subspace: "
              << reachableDimension << "\n";

    if (reachableDimension < static_cast<int>(A.size())) {
        std::cout << "Some state directions are not reachable from the actuator.\n";
    } else {
        std::cout << "The reachable subspace is the whole state space.\n";
    }

    return 0;
}
