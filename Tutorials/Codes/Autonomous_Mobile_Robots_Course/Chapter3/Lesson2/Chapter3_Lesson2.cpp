/*
Chapter 3 - Lesson 2: Curvature and Turning Radius Limits (C++)

Build note:
  This example uses Eigen (header-only). If you do not have Eigen,
  you can replace Eigen::Vector2d with a small custom struct.

Example compilation (Linux/macOS):
  g++ -O2 -std=c++17 Chapter3_Lesson2.cpp -I /usr/include/eigen3 -o Chapter3_Lesson2

This program:
  1) Computes discrete signed curvature along a polyline using a 3-point circumcircle formula.
  2) Checks steering curvature limit kappa_max = tan(delta_max)/L.
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

static double signed_curvature_three_points(const Vec2& p0, const Vec2& p1, const Vec2& p2) {
    Vec2 v01 = p1 - p0;
    Vec2 v02 = p2 - p0;
    Vec2 v12 = p2 - p1;

    double a = v12.norm();
    double b = v02.norm();
    double c = v01.norm();

    // Signed double area (2*A) in 2D: cross(v01, v02)
    double area2 = v01.x() * v02.y() - v01.y() * v02.x();

    double denom = a * b * c;
    if (denom < 1e-12) return 0.0;
    return 2.0 * area2 / denom; // since kappa = 4*A/(abc) and area2 = 2*A
}

static std::vector<double> curvature_polyline(const std::vector<Vec2>& P) {
    const int N = static_cast<int>(P.size());
    std::vector<double> kappa(N, 0.0);
    for (int i = 1; i < N - 1; ++i) {
        kappa[i] = signed_curvature_three_points(P[i-1], P[i], P[i+1]);
    }
    return kappa;
}

static double kappa_max_steer(double L, double delta_max_rad) {
    return std::tan(delta_max_rad) / L;
}

int main() {
    // Create a sample path
    const int N = 401;
    std::vector<Vec2> P;
    P.reserve(N);
    for (int i = 0; i < N; ++i) {
        double s = 12.0 * static_cast<double>(i) / static_cast<double>(N - 1);
        double x = s;
        double y = 1.2 * std::sin(0.7 * s) + 0.2 * std::sin(2.2 * s);
        P.emplace_back(x, y);
    }

    auto kappa = curvature_polyline(P);

    // Bicycle steering limit
    double L = 0.35;
    double delta_max = 28.0 * M_PI / 180.0;
    double kappa_max = kappa_max_steer(L, delta_max);

    bool ok = true;
    double kappa_peak = 0.0;
    for (double ki : kappa) {
        kappa_peak = std::max(kappa_peak, std::abs(ki));
        if (std::abs(ki) > kappa_max + 1e-12) ok = false;
    }

    std::cout << "kappa_max_steer = " << kappa_max << " 1/m\n";
    std::cout << "peak |kappa|    = " << kappa_peak << " 1/m\n";
    std::cout << "steering-feasible? " << (ok ? "YES" : "NO") << "\n";
    return 0;
}
