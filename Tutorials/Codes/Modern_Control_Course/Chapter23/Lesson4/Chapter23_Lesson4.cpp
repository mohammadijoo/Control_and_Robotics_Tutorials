// Chapter23_Lesson4.cpp
// Mapping time-domain specifications to desired poles and Ackermann pole placement.
// Compile: g++ -std=c++17 Chapter23_Lesson4.cpp -O2 -o Chapter23_Lesson4

#include <cmath>
#include <complex>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vec = std::vector<double>;
using Cx = std::complex<double>;

Matrix identity(int n) {
    Matrix I(n, Vec(n, 0.0));
    for (int i = 0; i < n; ++i) I[i][i] = 1.0;
    return I;
}

Matrix matmul(const Matrix& A, const Matrix& B) {
    int n = (int)A.size(), m = (int)B[0].size(), r = (int)B.size();
    Matrix C(n, Vec(m, 0.0));
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < r; ++k)
            for (int j = 0; j < m; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Vec matvec(const Matrix& A, const Vec& x) {
    Vec y(A.size(), 0.0);
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < x.size(); ++j) y[i] += A[i][j] * x[j];
    return y;
}

Matrix matadd(const Matrix& A, const Matrix& B, double scaleB = 1.0) {
    Matrix C = A;
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < A[0].size(); ++j) C[i][j] += scaleB * B[i][j];
    return C;
}

Matrix matscale(const Matrix& A, double s) {
    Matrix B = A;
    for (auto& row : B) for (double& v : row) v *= s;
    return B;
}

Matrix matpow(Matrix A, int p) {
    int n = (int)A.size();
    Matrix R = identity(n);
    while (p > 0) {
        if (p & 1) R = matmul(R, A);
        A = matmul(A, A);
        p >>= 1;
    }
    return R;
}

Matrix inverse(Matrix A) {
    int n = (int)A.size();
    Matrix aug(n, Vec(2 * n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) aug[i][j] = A[i][j];
        aug[i][n + i] = 1.0;
    }
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r)
            if (std::abs(aug[r][col]) > std::abs(aug[piv][col])) piv = r;
        if (std::abs(aug[piv][col]) < 1e-12) throw std::runtime_error("singular matrix");
        std::swap(aug[piv], aug[col]);
        double div = aug[col][col];
        for (double& v : aug[col]) v /= div;
        for (int r = 0; r < n; ++r) if (r != col) {
            double f = aug[r][col];
            for (int j = 0; j < 2 * n; ++j) aug[r][j] -= f * aug[col][j];
        }
    }
    Matrix inv(n, Vec(n, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) inv[i][j] = aug[i][n + j];
    return inv;
}

std::vector<Cx> polyFromRoots(const std::vector<Cx>& roots) {
    std::vector<Cx> c = {Cx(1.0, 0.0)};
    for (Cx r : roots) {
        std::vector<Cx> next(c.size() + 1, Cx(0.0, 0.0));
        for (size_t i = 0; i < c.size(); ++i) {
            next[i] += c[i];
            next[i + 1] += -r * c[i];
        }
        c = next;
    }
    return c;
}

double dampingRatioFromOvershoot(double percentOvershoot) {
    double m = percentOvershoot / 100.0;
    double L = std::log(m);
    return -L / std::sqrt(M_PI * M_PI + L * L);
}

std::vector<Cx> dominantPoles(double percentOvershoot, double settlingTime) {
    double zeta = dampingRatioFromOvershoot(percentOvershoot);
    double wn = 4.0 / (zeta * settlingTime);
    double sigma = zeta * wn;
    double wd = wn * std::sqrt(std::max(0.0, 1.0 - zeta * zeta));
    return {Cx(-sigma, wd), Cx(-sigma, -wd)};
}

Matrix controllabilityMatrix(const Matrix& A, const Vec& b) {
    int n = (int)A.size();
    Matrix W(n, Vec(n, 0.0));
    Vec col = b;
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) W[i][k] = col[i];
        col = matvec(A, col);
    }
    return W;
}

Vec ackermannGain(const Matrix& A, const Vec& b, const std::vector<Cx>& poles) {
    int n = (int)A.size();
    Matrix W = controllabilityMatrix(A, b);
    Matrix Winv = inverse(W);
    auto coeffCx = polyFromRoots(poles); // s^n + a_{n-1}s^{n-1}+...+a0

    Matrix phi = matpow(A, n);
    for (int i = 0; i < n; ++i) {
        int power = n - 1 - i;
        phi = matadd(phi, matscale(matpow(A, power), coeffCx[i + 1].real()));
    }
    Matrix temp = matmul(Winv, phi);
    Vec K(n, 0.0); // e_n^T * temp
    for (int j = 0; j < n; ++j) K[j] = temp[n - 1][j];
    return K;
}

int main() {
    Matrix A = {{0, 1, 0}, {0, 0, 1}, {-2, -3, -1}};
    Vec b = {0, 0, 1};

    auto poles = dominantPoles(10.0, 2.0);
    double sigma = -poles[0].real();
    poles.push_back(Cx(-6.0 * sigma, 0.0));

    Vec K = ackermannGain(A, b, poles);
    std::cout << "Desired poles:\n";
    for (auto p : poles) std::cout << p << "\n";
    std::cout << "K = [ ";
    for (double k : K) std::cout << k << " ";
    std::cout << "]\n";
    return 0;
}
