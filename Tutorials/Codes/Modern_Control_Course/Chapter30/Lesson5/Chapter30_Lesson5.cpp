// Chapter30_Lesson5.cpp
//
// Bridge lab for Modern Control, Chapter 30 Lesson 5.
// Implements an unconstrained finite-horizon MPC/LQR recursion from scratch
// for a sampled double integrator.
//
// Compile:
//   g++ -std=c++17 -O2 Chapter30_Lesson5.cpp -o Chapter30_Lesson5
//
// Run:
//   ./Chapter30_Lesson5

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct Vec2 {
    double x1;
    double x2;
};

struct Mat2 {
    double a11, a12, a21, a22;
};

Vec2 add(Vec2 a, Vec2 b) {
    return {a.x1 + b.x1, a.x2 + b.x2};
}

Vec2 sub(Vec2 a, Vec2 b) {
    return {a.x1 - b.x1, a.x2 - b.x2};
}

Vec2 scale(Vec2 a, double s) {
    return {s * a.x1, s * a.x2};
}

Vec2 mat_vec(Mat2 A, Vec2 x) {
    return {A.a11 * x.x1 + A.a12 * x.x2,
            A.a21 * x.x1 + A.a22 * x.x2};
}

Mat2 add(Mat2 A, Mat2 B) {
    return {A.a11 + B.a11, A.a12 + B.a12,
            A.a21 + B.a21, A.a22 + B.a22};
}

Mat2 sub(Mat2 A, Mat2 B) {
    return {A.a11 - B.a11, A.a12 - B.a12,
            A.a21 - B.a21, A.a22 - B.a22};
}

Mat2 mul(Mat2 A, Mat2 B) {
    return {
        A.a11 * B.a11 + A.a12 * B.a21,
        A.a11 * B.a12 + A.a12 * B.a22,
        A.a21 * B.a11 + A.a22 * B.a21,
        A.a21 * B.a12 + A.a22 * B.a22
    };
}

Mat2 transpose(Mat2 A) {
    return {A.a11, A.a21, A.a12, A.a22};
}

double dot(Vec2 a, Vec2 b) {
    return a.x1 * b.x1 + a.x2 * b.x2;
}

Vec2 row_times_mat(Vec2 row, Mat2 A) {
    return {row.x1 * A.a11 + row.x2 * A.a21,
            row.x1 * A.a12 + row.x2 * A.a22};
}

Mat2 outer(Vec2 a, Vec2 b) {
    return {a.x1 * b.x1, a.x1 * b.x2,
            a.x2 * b.x1, a.x2 * b.x2};
}

Vec2 mat_transpose_vec(Mat2 A, Vec2 x) {
    return {A.a11 * x.x1 + A.a21 * x.x2,
            A.a12 * x.x1 + A.a22 * x.x2};
}

struct GainResult {
    Vec2 K0;
    std::vector<Vec2> gains;
};

GainResult finite_horizon_lqr(Mat2 Ad, Vec2 Bd, Mat2 Q, double R,
                              Mat2 Qf, int horizon) {
    Mat2 P = Qf;
    std::vector<Vec2> backward;

    for (int k = horizon - 1; k >= 0; --k) {
        Vec2 PB = mat_vec(P, Bd);
        double S = R + dot(Bd, PB);
        Vec2 PA_col1 = mat_vec(P, {Ad.a11, Ad.a21});
        Vec2 PA_col2 = mat_vec(P, {Ad.a12, Ad.a22});

        Vec2 BtPA = {
            (Bd.x1 * PA_col1.x1 + Bd.x2 * PA_col1.x2) / S,
            (Bd.x1 * PA_col2.x1 + Bd.x2 * PA_col2.x2) / S
        };
        backward.push_back(BtPA);

        Mat2 BK = outer(Bd, BtPA);
        Mat2 Ad_minus_BK = sub(Ad, BK);
        P = add(Q, mul(transpose(Ad), mul(P, Ad_minus_BK)));
    }

    std::vector<Vec2> gains(backward.rbegin(), backward.rend());
    return {gains.front(), gains};
}

int main() {
    const double Ts = 0.05;

    // Sampled double integrator:
    // x_{k+1} = Ad x_k + Bd u_k
    Mat2 Ad{1.0, Ts, 0.0, 1.0};
    Vec2 Bd{0.5 * Ts * Ts, Ts};

    Mat2 Q{10.0, 0.0, 0.0, 1.0};
    double R = 0.25;
    Mat2 Qf{10.0, 0.0, 0.0, 1.0};

    GainResult result = finite_horizon_lqr(Ad, Bd, Q, R, Qf, 25);
    Vec2 K = result.K0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Finite-horizon MPC first gain K0 = ["
              << K.x1 << ", " << K.x2 << "]\n";

    Vec2 x{1.0, 0.0};
    double cost = 0.0;

    for (int k = 0; k < 80; ++k) {
        double u = -(K.x1 * x.x1 + K.x2 * x.x2);
        double stage = 10.0 * x.x1 * x.x1 + x.x2 * x.x2 + R * u * u;
        cost += stage;

        Vec2 next = add(mat_vec(Ad, x), scale(Bd, u));
        if (k % 10 == 0) {
            std::cout << "k=" << std::setw(2) << k
                      << "  x=[" << std::setw(10) << x.x1
                      << ", " << std::setw(10) << x.x2
                      << "]  u=" << std::setw(10) << u << "\n";
        }
        x = next;
    }

    std::cout << "Approximate accumulated cost = " << cost << "\n";
    std::cout << "Final state = [" << x.x1 << ", " << x.x2 << "]\n";
    return 0;
}
