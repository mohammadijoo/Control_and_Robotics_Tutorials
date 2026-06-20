/*
Chapter 3 — Nonholonomic Motion and Feasibility for AMR
Lesson 1 — Nonholonomic Constraints in Wheeled Robots (applied view)

Differential-drive kinematic simulation and constraint check.

Build (example):
  g++ -O2 -std=c++17 Chapter3_Lesson1.cpp -o Chapter3_Lesson1

Optional robotics-adjacent dependency:
  - Eigen (linear algebra) can be used, but this example stays standard-library only.
*/

#include <cmath>
#include <iostream>
#include <vector>

struct SimResult {
    std::vector<double> t, x, y, th, v, w, res;
};

static inline void diff_drive_twist(double omega_l, double omega_r, double r, double b,
                                    double& v_out, double& w_out) {
    v_out = 0.5 * r * (omega_r + omega_l);
    w_out = 0.5 * r * (omega_r - omega_l) / b;
}

static inline double lateral_constraint_residual(double x_dot, double y_dot, double theta) {
    // -sin(theta) x_dot + cos(theta) y_dot = 0
    return -std::sin(theta) * x_dot + std::cos(theta) * y_dot;
}

static inline void unicycle_step(double& x, double& y, double& theta, double v, double w, double dt) {
    x += dt * v * std::cos(theta);
    y += dt * v * std::sin(theta);
    theta += dt * w;

    // wrap to [-pi, pi]
    theta = std::fmod(theta + M_PI, 2.0 * M_PI);
    if (theta < 0) theta += 2.0 * M_PI;
    theta -= M_PI;
}

static inline void wheel_profile(double t, double& omega_l, double& omega_r) {
    if (t < 4.0) {
        omega_l = 6.0; omega_r = 6.0;
    } else if (t < 8.0) {
        omega_l = 3.0; omega_r = 7.0;
    } else if (t < 12.0) {
        omega_l = 7.0; omega_r = 3.0;
    } else {
        omega_l = 5.0; omega_r = 5.0;
    }
}

SimResult run_sim(double T = 16.0, double dt = 0.01, double r = 0.10, double b = 0.22) {
    const int N = static_cast<int>(std::floor(T / dt)) + 1;

    SimResult s;
    s.t.resize(N); s.x.resize(N); s.y.resize(N); s.th.resize(N);
    s.v.resize(N); s.w.resize(N); s.res.resize(N);

    double x = 0.0, y = 0.0, th = 0.0;

    for (int k = 0; k < N; ++k) {
        const double tk = k * dt;
        s.t[k] = tk;

        double om_l = 0.0, om_r = 0.0;
        wheel_profile(tk, om_l, om_r);

        double v = 0.0, w = 0.0;
        diff_drive_twist(om_l, om_r, r, b, v, w);

        const double x_dot = v * std::cos(th);
        const double y_dot = v * std::sin(th);

        s.x[k] = x; s.y[k] = y; s.th[k] = th;
        s.v[k] = v; s.w[k] = w;
        s.res[k] = lateral_constraint_residual(x_dot, y_dot, th);

        if (k < N - 1) {
            unicycle_step(x, y, th, v, w, dt);
        }
    }
    return s;
}

int main() {
    auto sim = run_sim();

    double max_abs = 0.0;
    for (double e : sim.res) max_abs = std::max(max_abs, std::abs(e));

    std::cout << "max |constraint residual| = " << max_abs << "\n";
    std::cout << "final pose: x=" << sim.x.back() << " y=" << sim.y.back()
              << " theta=" << sim.th.back() << "\n";

    // For plotting, export CSV if needed (left as exercise)
    return 0;
}
