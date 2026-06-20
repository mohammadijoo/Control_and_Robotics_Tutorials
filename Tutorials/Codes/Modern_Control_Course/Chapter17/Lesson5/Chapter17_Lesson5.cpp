// Chapter17_Lesson5.cpp
// Practical Considerations in Choosing Canonical Forms
// Compile: g++ -std=c++17 Chapter17_Lesson5.cpp -o Chapter17_Lesson5

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

Matrix matmul(const Matrix& A, const Matrix& B) {
    int r = (int)A.size(), k = (int)B.size(), c = (int)B[0].size();
    Matrix M(r, std::vector<double>(c, 0.0));
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            for (int t = 0; t < k; ++t)
                M[i][j] += A[i][t] * B[t][j];
    return M;
}

Matrix transpose(const Matrix& A) {
    int r = (int)A.size(), c = (int)A[0].size();
    Matrix T(c, std::vector<double>(r));
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            T[j][i] = A[i][j];
    return T;
}

Matrix inverse(Matrix A) {
    int n = (int)A.size();
    Matrix Aug(n, std::vector<double>(2 * n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) Aug[i][j] = A[i][j];
        Aug[i][n + i] = 1.0;
    }
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int i = col + 1; i < n; ++i)
            if (std::abs(Aug[i][col]) > std::abs(Aug[pivot][col])) pivot = i;
        if (std::abs(Aug[pivot][col]) < 1e-12) throw std::runtime_error("Singular matrix");
        std::swap(Aug[pivot], Aug[col]);
        double div = Aug[col][col];
        for (int j = 0; j < 2 * n; ++j) Aug[col][j] /= div;
        for (int i = 0; i < n; ++i) {
            if (i == col) continue;
            double factor = Aug[i][col];
            for (int j = 0; j < 2 * n; ++j) Aug[i][j] -= factor * Aug[col][j];
        }
    }
    Matrix Inv(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Inv[i][j] = Aug[i][n + j];
    return Inv;
}

double frobeniusNorm(const Matrix& A) {
    double s = 0.0;
    for (const auto& row : A)
        for (double v : row) s += v * v;
    return std::sqrt(s);
}

double frobeniusCondition(const Matrix& A) {
    return frobeniusNorm(A) * frobeniusNorm(inverse(A));
}

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    int n = (int)A.size();
    Matrix O;
    Matrix Ak = eye(n);
    for (int k = 0; k < n; ++k) {
        Matrix row = matmul(C, Ak);
        O.push_back(row[0]);
        Ak = matmul(Ak, A);
    }
    return O;
}

void printMatrix(const std::string& name, const Matrix& A) {
    std::cout << name << "\n";
    for (const auto& row : A) {
        for (double v : row) std::cout << std::setw(12) << std::setprecision(6) << std::fixed << v << " ";
        std::cout << "\n";
    }
}

Matrix diag(const std::vector<double>& d) {
    int n = (int)d.size();
    Matrix D(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) D[i][i] = d[i];
    return D;
}

int main() {
    Matrix A0 = {{0, 1, 0}, {0, 0, 1}, {-6, -11, -6}};
    Matrix B0 = {{0}, {0}, {1}};
    Matrix C0 = {{1, 0, 0}};
    Matrix S = diag({1.0, 0.05, 20.0});
    Matrix A = matmul(matmul(S, A0), inverse(S));
    Matrix B = matmul(S, B0);
    Matrix C = matmul(C0, inverse(S));

    Matrix Ao = transpose(A0);
    Matrix Co = {{0, 0, 1}};
    Matrix O = observabilityMatrix(A, C);
    Matrix Oo = observabilityMatrix(Ao, Co);
    Matrix T_ocf = matmul(inverse(O), Oo);
    Matrix Ti_ocf = inverse(T_ocf);
    Matrix A_ocf = matmul(matmul(Ti_ocf, A), T_ocf);
    Matrix C_ocf = matmul(C, T_ocf);

    // Modal eigenvectors are known for the companion model: [1, lambda, lambda^2]^T.
    // For A = S A0 S^{-1}, the corresponding eigenvectors are S times those vectors.
    std::vector<double> lam = {-1.0, -2.0, -3.0};
    Matrix V(3, std::vector<double>(3));
    for (int j = 0; j < 3; ++j) {
        double l = lam[j];
        Matrix v0 = {{1.0}, {l}, {l * l}};
        Matrix v = matmul(S, v0);
        for (int i = 0; i < 3; ++i) V[i][j] = v[i][0];
    }
    Matrix A_modal = matmul(matmul(inverse(V), A), V);

    printMatrix("Physical/scaled A:", A);
    printMatrix("Observable canonical A:", A_ocf);
    printMatrix("Observable canonical C:", C_ocf);
    std::cout << "OCF transform condition proxy = " << frobeniusCondition(T_ocf) << "\n\n";
    printMatrix("Modal A:", A_modal);
    std::cout << "Modal eigenvector condition proxy = " << frobeniusCondition(V) << "\n";

    double kModal = frobeniusCondition(V);
    double kOcf = frobeniusCondition(T_ocf);
    if (std::min(kModal, kOcf) > 5000.0)
        std::cout << "Recommendation: both canonical transforms are poorly conditioned; keep physical/scaled coordinates or use an orthogonal Schur form.\n";
    else if (kModal < kOcf)
        std::cout << "Recommendation: modal coordinates are cleaner for this example.\n";
    else
        std::cout << "Recommendation: OCF is acceptable for observer algebra in this example.\n";
    return 0;
}
