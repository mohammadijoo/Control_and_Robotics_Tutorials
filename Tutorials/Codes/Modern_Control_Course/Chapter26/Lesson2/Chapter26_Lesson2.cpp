// Chapter26_Lesson2.cpp
// Modern Control - State augmentation with integral of tracking error
// Self-contained SISO example using Ackermann's formula for the augmented model.
// Compile: g++ -std=c++17 Chapter26_Lesson2.cpp -O2 -o Chapter26_Lesson2

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Mat3 = std::array<std::array<double, 3>, 3>;
using Vec3 = std::array<double, 3>;

Mat3 eye3() {
    return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}

Mat3 add(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Mat3 scale(const Mat3& A, double s) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[i][j] = s * A[i][j];
    return C;
}

Mat3 mul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Vec3 mat_vec(const Mat3& A, const Vec3& x) {
    Vec3 y{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            y[i] += A[i][j] * x[j];
    return y;
}

double det3(const Mat3& M) {
    return M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
         - M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0])
         + M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);
}

Mat3 inverse3(const Mat3& M) {
    double d = det3(M);
    if (std::abs(d) < 1e-12) {
        throw std::runtime_error("Matrix is singular.");
    }
    Mat3 inv{};
    inv[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) / d;
    inv[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) / d;
    inv[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) / d;
    inv[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) / d;
    inv[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) / d;
    inv[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) / d;
    inv[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) / d;
    inv[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) / d;
    inv[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) / d;
    return inv;
}

Vec3 row_times_mat(const Vec3& r, const Mat3& M) {
    Vec3 y{};
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
            y[j] += r[k] * M[k][j];
    return y;
}

void print_vec(const char* name, const Vec3& v) {
    std::cout << name << " = [ ";
    for (double x : v) std::cout << std::setw(10) << x << " ";
    std::cout << "]\n";
}

int main() {
    // Plant: x_dot = A x + B u, y = C x.
    // A = [[0, 1], [-2, -3]], B = [[0], [1]], C = [1, 0].
    // Integral state q_dot = r - y.
    Mat3 Aa = {{{0, 1, 0},
                {-2, -3, 0},
                {-1, 0, 0}}};
    Vec3 Ba = {0, 1, 0};

    // Controllability matrix Cc = [Ba, Aa*Ba, Aa^2*Ba].
    Vec3 c1 = Ba;
    Vec3 c2 = mat_vec(Aa, Ba);
    Vec3 c3 = mat_vec(Aa, c2);
    Mat3 Cc = {{{c1[0], c2[0], c3[0]},
                {c1[1], c2[1], c3[1]},
                {c1[2], c2[2], c3[2]}}};

    std::cout << "det controllability(Aa,Ba) = " << det3(Cc) << "\n";

    // Desired poles: -2, -3, -4 -> phi(s)=s^3 + 9s^2 + 26s + 24.
    Mat3 Aa2 = mul(Aa, Aa);
    Mat3 Aa3 = mul(Aa2, Aa);
    Mat3 phiAa = add(add(Aa3, scale(Aa2, 9.0)), add(scale(Aa, 26.0), scale(eye3(), 24.0)));

    // Ackermann formula: K = e_n^T Cc^{-1} phi(Aa).
    Vec3 en = {0, 0, 1};
    Mat3 CcInv = inverse3(Cc);
    Vec3 temp = row_times_mat(en, CcInv);
    Vec3 Kaug = row_times_mat(temp, phiAa);

    print_vec("Kaug for u = -Kaug [x1 x2 q]^T", Kaug);
    std::cout << "Equivalent integral gain Ki in u = -Kx*x + Ki*q is Ki = "
              << -Kaug[2] << "\n";

    return 0;
}
