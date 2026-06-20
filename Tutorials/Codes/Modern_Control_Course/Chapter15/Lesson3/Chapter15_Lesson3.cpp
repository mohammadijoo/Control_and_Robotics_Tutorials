// Chapter15_Lesson3.cpp
// Finite-horizon observability Gramian and qualitative coordinate-sensor scoring.
// From-scratch implementation for small teaching examples.
//
// Compile:
//   g++ -std=c++17 Chapter15_Lesson3.cpp -O2 -o Chapter15_Lesson3

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

Matrix zeros(int r, int c) {
    return Matrix(r, std::vector<double>(c, 0.0));
}

Matrix transpose(const Matrix& A) {
    int r = static_cast<int>(A.size());
    int c = static_cast<int>(A[0].size());
    Matrix T = zeros(c, r);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            T[j][i] = A[i][j];
    return T;
}

Matrix add(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int c = static_cast<int>(A[0].size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Matrix scale(const Matrix& A, double s) {
    Matrix B = A;
    for (auto& row : B)
        for (double& v : row)
            v *= s;
    return B;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int r = static_cast<int>(A.size());
    int c = static_cast<int>(B[0].size());
    int inner = static_cast<int>(B.size());
    Matrix C = zeros(r, c);
    for (int i = 0; i < r; ++i)
        for (int k = 0; k < inner; ++k)
            for (int j = 0; j < c; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix sensorMatrix(const std::vector<int>& sensors, int n) {
    Matrix C = zeros(static_cast<int>(sensors.size()), n);
    for (int r = 0; r < static_cast<int>(sensors.size()); ++r)
        C[r][sensors[r]] = 1.0;
    return C;
}

Matrix gramianRhs(const Matrix& A, const Matrix& W, const Matrix& Q) {
    Matrix AT = transpose(A);
    return add(add(multiply(AT, W), multiply(W, A)), Q);
}

Matrix finiteHorizonGramian(const Matrix& A, const Matrix& C, double T, int steps) {
    int n = static_cast<int>(A.size());
    Matrix W = zeros(n, n);
    Matrix Q = multiply(transpose(C), C);
    double h = T / static_cast<double>(steps);

    for (int s = 0; s < steps; ++s) {
        Matrix k1 = gramianRhs(A, W, Q);
        Matrix k2 = gramianRhs(A, add(W, scale(k1, 0.5 * h)), Q);
        Matrix k3 = gramianRhs(A, add(W, scale(k2, 0.5 * h)), Q);
        Matrix k4 = gramianRhs(A, add(W, scale(k3, h)), Q);

        Matrix increment = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
        W = add(W, scale(increment, h / 6.0));
    }

    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            double avg = 0.5 * (W[i][j] + W[j][i]);
            W[i][j] = avg;
            W[j][i] = avg;
        }

    return W;
}

double trace(const Matrix& A) {
    double s = 0.0;
    for (int i = 0; i < static_cast<int>(A.size()); ++i) s += A[i][i];
    return s;
}

double det3(const Matrix& A) {
    return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
         - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
         + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
}

double regularizedLogDet3(const Matrix& W, double eps) {
    Matrix R = W;
    for (int i = 0; i < 3; ++i) R[i][i] += eps;
    double d = det3(R);
    if (d <= 0.0) return -std::numeric_limits<double>::infinity();
    return std::log(d);
}

void printScore(const std::string& name, const Matrix& W) {
    std::cout << std::setw(8) << name
              << "  trace=" << std::setw(12) << trace(W)
              << "  logdet_eps=" << std::setw(12) << regularizedLogDet3(W, 1e-8)
              << "  det_eps=" << std::setw(12) << std::exp(regularizedLogDet3(W, 1e-8))
              << "\n";
}

int main() {
    Matrix A = {
        {0.0, 1.0, 0.0},
        {-2.0, -0.45, 0.8},
        {0.0, -0.7, -1.25}
    };

    const int n = 3;
    const double T = 6.0;
    const int steps = 4000;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Single-sensor scores\n";
    for (int i = 0; i < n; ++i) {
        Matrix C = sensorMatrix({i}, n);
        Matrix W = finiteHorizonGramian(A, C, T, steps);
        printScore("x" + std::to_string(i + 1), W);
    }

    std::cout << "\nTwo-sensor scores\n";
    double bestScore = -std::numeric_limits<double>::infinity();
    std::string bestName;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            Matrix C = sensorMatrix({i, j}, n);
            Matrix W = finiteHorizonGramian(A, C, T, steps);
            std::string name = "x" + std::to_string(i + 1) + ",x" + std::to_string(j + 1);
            double score = regularizedLogDet3(W, 1e-8);
            printScore(name, W);
            if (score > bestScore) {
                bestScore = score;
                bestName = name;
            }
        }
    }

    std::cout << "\nBest qualitative two-sensor set by log-det: " << bestName << "\n";
    return 0;
}
