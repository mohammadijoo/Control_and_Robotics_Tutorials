/*
Chapter4_Lesson2.cpp
Autonomous Mobile Robots — Chapter 4 Lesson 2
Slip, Skid, and Terrain Interaction Models

This C++ example mirrors the Python script:
- Hard-ground combined-slip saturation with a friction ellipse
- Simple 1D wheel + vehicle simulation

Suggested libraries in robotics stacks:
  - Eigen (linear algebra) for larger models
  - ROS 2 (rclcpp) for real-time integration (not used here)

Build (example):
  g++ -O2 -std=c++17 Chapter4_Lesson2.cpp -o lesson2

*/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static double slip_ratio(double vx, double omega, double R, double eps = 1e-6) {
    const double denom = std::max({std::abs(vx), std::abs(R * omega), eps});
    return (R * omega - vx) / denom;
}

static double friction_ellipse_scale(double Fx, double Fy, double mu, double Fz) {
    const double cap = std::max(mu * Fz, 0.0);
    if (cap <= 0.0) return 0.0;
    const double nx = Fx / cap;
    const double ny = Fy / cap;
    const double r2 = nx * nx + ny * ny;
    if (r2 <= 1.0) return 1.0;
    return 1.0 / std::sqrt(r2);
}

static double hard_ground_Fx(double vx, double omega, double R, double Fz, double mu,
                             double Ck = 15000.0) {
    const double kappa = slip_ratio(vx, omega, R);
    const double Fx_lin = Ck * kappa;
    // 1D: ignore lateral force, so ellipse reduces to |Fx| <= mu Fz
    const double cap = std::max(mu * Fz, 0.0);
    return std::clamp(Fx_lin, -cap, cap);
}

static double soft_soil_Fx(double kappa, double Fz, double mu_peak = 0.55, double k_shape = 10.0) {
    const double s = (kappa > 0.0) ? 1.0 : (kappa < 0.0 ? -1.0 : 0.0);
    const double mu_eff = mu_peak * (1.0 - std::exp(-k_shape * std::abs(kappa))) * s;
    return Fz * mu_eff;
}

struct SimResult {
    std::vector<double> t, vx, omega, kappa, Fx;
};

static SimResult simulate_1d(double T_cmd, const std::string& terrain, double t_end = 4.0, double dt = 1e-3) {
    // Parameters
    const double m = 25.0;
    const double R = 0.10;
    const double Iw = 0.05;
    const double bw = 0.02;
    const double g = 9.81;
    const double Fz = 0.25 * m * g;
    const double Crr = 0.02;
    const double Frr = Crr * Fz;
    const double mu = 0.8;

    const int N = static_cast<int>(t_end / dt) + 1;

    SimResult out;
    out.t.resize(N);
    out.vx.assign(N, 0.0);
    out.omega.assign(N, 0.0);
    out.kappa.assign(N, 0.0);
    out.Fx.assign(N, 0.0);

    for (int i = 0; i < N; ++i) out.t[i] = i * dt;

    for (int i = 0; i < N - 1; ++i) {
        double Fx_i = 0.0;
        double k_i = slip_ratio(out.vx[i], out.omega[i], R);

        if (terrain == "hard") {
            Fx_i = hard_ground_Fx(out.vx[i], out.omega[i], R, Fz, mu);
        } else if (terrain == "soft") {
            Fx_i = soft_soil_Fx(k_i, Fz);
            Fx_i -= 0.015 * Fz * ((out.vx[i] > 0.0) ? 1.0 : (out.vx[i] < 0.0 ? -1.0 : 0.0));
        } else {
            throw std::runtime_error("terrain must be 'hard' or 'soft'");
        }

        const double ax = (Fx_i - Frr) / m;
        out.vx[i + 1] = out.vx[i] + dt * ax;

        const double domega = (T_cmd - R * Fx_i - bw * out.omega[i]) / Iw;
        out.omega[i + 1] = out.omega[i] + dt * domega;

        out.Fx[i] = Fx_i;
        out.kappa[i] = k_i;
    }

    out.Fx[N - 1] = out.Fx[N - 2];
    out.kappa[N - 1] = out.kappa[N - 2];
    return out;
}

int main() {
    try {
        auto hard = simulate_1d(12.0, "hard");
        auto soft = simulate_1d(12.0, "soft");

        // Print a small summary (final values)
        const auto idx = static_cast<int>(hard.t.size()) - 1;
        std::cout << "Final (hard): vx=" << hard.vx[idx] << " m/s, kappa=" << hard.kappa[idx] << ", Fx=" << hard.Fx[idx] << " N\n";
        std::cout << "Final (soft): vx=" << soft.vx[idx] << " m/s, kappa=" << soft.kappa[idx] << ", Fx=" << soft.Fx[idx] << " N\n";

        std::cout << "Tip: export vectors to CSV if you want to plot externally.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
