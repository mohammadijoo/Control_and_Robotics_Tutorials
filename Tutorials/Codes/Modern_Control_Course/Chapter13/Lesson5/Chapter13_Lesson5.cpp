// Chapter13_Lesson5.cpp
// Observability examples using only the C++ standard library.
// Compile: g++ -std=c++17 Chapter13_Lesson5.cpp -o Chapter13_Lesson5

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix multiply(const Matrix& A, const Matrix& B) {
    int m = static_cast<int>(A.size());
    int p = static_cast<int>(A[0].size());
    int n = static_cast<int>(B[0].size());
    Matrix R(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; ++i) {
        for (int k = 0; k < p; ++k) {
            for (int j = 0; j < n; ++j) {
                R[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return R;
}

Matrix identity(int n) {
    Matrix I(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix vstack(const std::vector<Matrix>& blocks) {
    Matrix R;
    for (const auto& B : blocks) {
        for (const auto& row : B) R.push_back(row);
    }
    return R;
}

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = static_cast<int>(A.size());
    Matrix Ak = identity(n);
    std::vector<Matrix> blocks;
    for (int k = 0; k < n; ++k) {
        blocks.push_back(multiply(C, Ak));
        Ak = multiply(Ak, A);
    }
    return vstack(blocks);
}

void printMatrix(const Matrix& M, const std::string& name) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double x : row) {
            std::cout << std::setw(12) << std::setprecision(6) << x << " ";
        }
        std::cout << "\n";
    }
}

int rankByGaussianElimination(Matrix M, double tol = 1e-10) {
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
        double div = M[rank][col];
        for (int j = col; j < cols; ++j) M[rank][j] /= div;

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

void classify(const std::string& name, const Matrix& A, const Matrix& C) {
    Matrix O = observabilityMatrix(A, C);
    int n = static_cast<int>(A.size());
    int r = rankByGaussianElimination(O);

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << name << "\n";
    printMatrix(A, "A");
    printMatrix(C, "C");
    printMatrix(O, "O");
    std::cout << "rank(O) = " << r << " out of n = " << n << "\n";
    if (r == n) std::cout << "Conclusion: observable.\n";
    else std::cout << "Conclusion: unobservable. Inspect hidden modes for detectability.\n";
}

int main() {
    Matrix A1 = {{0.0, 1.0}, {-2.0, -3.0}};
    Matrix C1 = {{1.0, 0.0}};
    classify("Example 1: observable second-order system", A1, C1);

    Matrix A2 = {{-1.0, 0.0}, {0.0, 2.0}};
    Matrix C2 = {{1.0, 0.0}};
    classify("Example 2: unobservable and not detectable", A2, C2);

    Matrix A3 = {{-1.0, 0.0}, {0.0, -4.0}};
    Matrix C3 = {{1.0, 0.0}};
    classify("Example 3: unobservable but detectable", A3, C3);

    Matrix A4 = {{0.0, 1.0}, {-6.0, -5.0}};
    Matrix C4a = {{0.0, 1.0}};
    Matrix C4b = {{1.0, 0.0}};
    classify("Example 4a: velocity sensor", A4, C4a);
    classify("Example 4b: position sensor", A4, C4b);

    return 0;
}
