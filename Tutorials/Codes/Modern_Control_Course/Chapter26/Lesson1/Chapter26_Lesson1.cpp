/*
Chapter26_Lesson1.cpp

Need for steady-state accuracy in a state-space framework.
Scratch 2-state implementation without external dependencies.

Compile:
  g++ -std=c++17 Chapter26_Lesson1.cpp -o Chapter26_Lesson1

Run:
  ./Chapter26_Lesson1
*/

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct Mat2 {
    double a11, a12, a21, a22;
};

struct Vec2 {
    double x1, x2;
};

Mat2 subtractBK(const Mat2& A, const Vec2& B, const Vec2& K) {
    return {
        A.a11 - B.x1 * K.x1, A.a12 - B.x1 * K.x2,
        A.a21 - B.x2 * K.x1, A.a22 - B.x2 * K.x2
    };
}

Mat2 inverse2(const Mat2& M) {
    double det = M.a11 * M.a22 - M.a12 * M.a21;
    if (std::abs(det) < 1e-12) {
        throw std::runtime_error("Matrix is singular or nearly singular.");
    }
    return {M.a22 / det, -M.a12 / det, -M.a21 / det, M.a11 / det};
}

Vec2 matVec(const Mat2& M, const Vec2& v) {
    return {M.a11 * v.x1 + M.a12 * v.x2, M.a21 * v.x1 + M.a22 * v.x2};
}

double dot(const Vec2& c, const Vec2& x) {
    return c.x1 * x.x1 + c.x2 * x.x2;
}

Vec2 rhs(const Mat2& Acl, const Vec2& B, const Vec2& x, double inputTotal) {
    Vec2 ax = matVec(Acl, x);
    return {ax.x1 + B.x1 * inputTotal, ax.x2 + B.x2 * inputTotal};
}

Vec2 rk4Step(const Mat2& Acl, const Vec2& B, const Vec2& x, double inputTotal, double h) {
    Vec2 k1 = rhs(Acl, B, x, inputTotal);
    Vec2 x2{x.x1 + 0.5 * h * k1.x1, x.x2 + 0.5 * h * k1.x2};
    Vec2 k2 = rhs(Acl, B, x2, inputTotal);
    Vec2 x3{x.x1 + 0.5 * h * k2.x1, x.x2 + 0.5 * h * k2.x2};
    Vec2 k3 = rhs(Acl, B, x3, inputTotal);
    Vec2 x4{x.x1 + h * k3.x1, x.x2 + h * k3.x2};
    Vec2 k4 = rhs(Acl, B, x4, inputTotal);

    return {
        x.x1 + (h / 6.0) * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1),
        x.x2 + (h / 6.0) * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2)
    };
}

double simulateFinalY(const Mat2& Acl, const Vec2& B, const Vec2& C,
                      double nbar, double reference, double disturbance) {
    Vec2 x{0.0, 0.0};
    double h = 0.001;
    double tFinal = 8.0;
    int steps = static_cast<int>(tFinal / h);
    double inputTotal = nbar * reference + disturbance;

    for (int i = 0; i < steps; ++i) {
        x = rk4Step(Acl, B, x, inputTotal, h);
    }
    return dot(C, x);
}

int main() {
    Mat2 A{0.0, 1.0, -2.0, -3.0};
    Vec2 B{0.0, 1.0};
    Vec2 C{1.0, 0.0};
    Vec2 K{4.0, 2.0};

    Mat2 Acl = subtractBK(A, B, K);
    Mat2 invAcl = inverse2(Acl);
    Vec2 invAclB = matVec(invAcl, B);

    double dcClosed = -dot(C, invAclB);
    double nbar = 1.0 / dcClosed;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Closed-loop DC map from prefilter input v to y: " << dcClosed << "\n";
    std::cout << "Required static prefilter Nbar: " << nbar << "\n\n";

    struct CaseData {
        std::string label;
        double nbar;
        double reference;
        double disturbance;
    };

    std::vector<CaseData> cases = {
        {"Nbar = 1, no disturbance", 1.0, 1.0, 0.0},
        {"Nbar = computed, no disturbance", nbar, 1.0, 0.0},
        {"Nbar = computed, disturbance d = 0.2", nbar, 1.0, 0.2}
    };

    for (const auto& item : cases) {
        double yFinal = simulateFinalY(Acl, B, C, item.nbar, item.reference, item.disturbance);
        std::cout << item.label
                  << ": final y approximately " << yFinal
                  << ", final error " << (item.reference - yFinal) << "\n";
    }

    return 0;
}
