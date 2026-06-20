// Chapter18_Lesson2.cpp
// From-scratch rank/nullity diagnostics for Jordan block sizes.
// This is educational code, not a numerically robust Jordan-form algorithm.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix identity(int n) {
    Matrix I(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix subtractLambdaI(const Matrix& A, double lambda) {
    Matrix B = A;
    for (int i = 0; i < static_cast<int>(A.size()); ++i) {
        B[i][i] -= lambda;
    }
    return B;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    int m = static_cast<int>(B[0].size());
    int p = static_cast<int>(B.size());
    Matrix C(n, std::vector<double>(m, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < p; ++k) {
            for (int j = 0; j < m; ++j) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

int rankGaussian(Matrix A, double tol = 1e-10) {
    int rows = static_cast<int>(A.size());
    int cols = static_cast<int>(A[0].size());
    int r = 0;

    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i) {
            if (std::fabs(A[i][c]) > std::fabs(A[pivot][c])) {
                pivot = i;
            }
        }
        if (std::fabs(A[pivot][c]) <= tol) continue;

        std::swap(A[r], A[pivot]);
        double div = A[r][c];
        for (int j = c; j < cols; ++j) A[r][j] /= div;

        for (int i = 0; i < rows; ++i) {
            if (i == r) continue;
            double factor = A[i][c];
            for (int j = c; j < cols; ++j) {
                A[i][j] -= factor * A[r][j];
            }
        }
        ++r;
    }
    return r;
}

std::vector<int> nullitySequence(const Matrix& A, double lambda, int maxPower) {
    int n = static_cast<int>(A.size());
    Matrix N = subtractLambdaI(A, lambda);
    Matrix Nk = identity(n);

    std::vector<int> nullities;
    nullities.push_back(0);

    for (int k = 1; k <= maxPower; ++k) {
        Nk = multiply(Nk, N);
        int rank = rankGaussian(Nk);
        nullities.push_back(n - rank);
    }
    return nullities;
}

std::vector<int> blockSizesFromNullities(const std::vector<int>& nullities,
                                         int algebraicMultiplicity) {
    std::vector<int> trimmed;
    trimmed.push_back(nullities[0]);

    for (size_t i = 1; i < nullities.size(); ++i) {
        trimmed.push_back(nullities[i]);
        if (nullities[i] == algebraicMultiplicity) break;
    }

    std::vector<int> blocksAtLeast;
    for (size_t k = 1; k < trimmed.size(); ++k) {
        blocksAtLeast.push_back(trimmed[k] - trimmed[k - 1]);
    }
    blocksAtLeast.push_back(0);

    std::vector<int> sizes;
    for (size_t k = 1; k < blocksAtLeast.size(); ++k) {
        int exactCount = blocksAtLeast[k - 1] - blocksAtLeast[k];
        for (int c = 0; c < exactCount; ++c) {
            sizes.push_back(static_cast<int>(k));
        }
    }

    std::sort(sizes.rbegin(), sizes.rend());
    return sizes;
}

void printVector(const std::vector<int>& v) {
    std::cout << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    std::cout << "]";
}

int main() {
    Matrix J = {
        { 2, 1, 0, 0,  0, 0},
        { 0, 2, 1, 0,  0, 0},
        { 0, 0, 2, 0,  0, 0},
        { 0, 0, 0, 2,  0, 0},
        { 0, 0, 0, 0, -1, 1},
        { 0, 0, 0, 0,  0,-1}
    };

    std::vector<double> eigenvalues = {2.0, -1.0};
    std::vector<int> algebraicMultiplicities = {4, 2};

    for (size_t idx = 0; idx < eigenvalues.size(); ++idx) {
        double lambda = eigenvalues[idx];
        int alg = algebraicMultiplicities[idx];

        std::vector<int> nullities = nullitySequence(J, lambda, 6);
        std::vector<int> sizes = blockSizesFromNullities(nullities, alg);

        std::cout << "lambda = " << lambda << "\n";
        std::cout << "nullities n_k = ";
        printVector(nullities);
        std::cout << "\nJordan block sizes = ";
        printVector(sizes);
        std::cout << "\n\n";
    }

    std::cout << "Expected: lambda 2 has block sizes [3, 1]; lambda -1 has [2].\n";
    return 0;
}
