// Chapter15_Lesson2.cpp
// Stanley Controller for Ground Vehicles (kinematic bicycle model, educational)
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter15_Lesson2.cpp -o stanley
//
// Notes:
// - For ROS2 integration, compute psi_ref and e_y from nav_msgs::msg::Path
//   and publish ackermann_msgs::msg::AckermannDriveStamped steering_angle.

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

static double wrap_angle(double a) {
    const double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2.0 * pi);
    if (a < 0) a += 2.0 * pi;
    return a - pi;
}

struct State {
    double x{0.0};
    double y{0.0};
    double psi{0.0}; // yaw [rad]
    double v{0.0};   // speed [m/s]
};

struct Params {
    double L{2.7};
    double k{1.4};
    double v0{0.5};
    double max_steer{30.0 * M_PI / 180.0};
};

static void make_s_path(std::vector<double>& xr, std::vector<double>& yr, int n = 400) {
    xr.resize(n);
    yr.resize(n);
    const double x0 = 0.0, x1 = 50.0;
    for (int i = 0; i < n; ++i) {
        double x = x0 + (x1 - x0) * (static_cast<double>(i) / (n - 1));
        xr[i] = x;
        yr[i] = 2.5 * std::sin(0.18 * x) + 1.0 * std::sin(0.04 * x);
    }
}

static int nearest_point_index(double px, double py, const std::vector<double>& xr, const std::vector<double>& yr) {
    // Brute-force nearest vertex (simple). For production, project onto segments / k-d tree.
    double best = std::numeric_limits<double>::infinity();
    int best_i = 0;
    for (int i = 0; i < static_cast<int>(xr.size()); ++i) {
        double dx = px - xr[i];
        double dy = py - yr[i];
        double d2 = dx*dx + dy*dy;
        if (d2 < best) { best = d2; best_i = i; }
    }
    return best_i;
}

static double heading_from_path(int i, const std::vector<double>& xr, const std::vector<double>& yr) {
    int n = static_cast<int>(xr.size());
    int j = std::min(i + 1, n - 1);
    int k = std::max(i - 1, 0);
    double dx = xr[j] - xr[k];
    double dy = yr[j] - yr[k];
    return std::atan2(dy, dx);
}

static double stanley_control(const State& s,
                              const std::vector<double>& xr,
                              const std::vector<double>& yr,
                              const Params& p,
                              double& e_y_out,
                              double& e_psi_out) {

    int i = nearest_point_index(s.x, s.y, xr, yr);
    double psi_ref = heading_from_path(i, xr, yr);

    // Convention: e_psi = psi_ref - psi (desired minus actual)
    double e_psi = wrap_angle(psi_ref - s.psi);

    // Signed cross-track: positive if vehicle is left of the path tangent
    double tx = std::cos(psi_ref);
    double ty = std::sin(psi_ref);
    double vx = s.x - xr[i];
    double vy = s.y - yr[i];
    double cross = tx * vy - ty * vx;
    double e_y = std::copysign(std::hypot(vx, vy), cross);

    double delta = e_psi + std::atan2(p.k * e_y, s.v + p.v0);
    if (delta > p.max_steer) delta = p.max_steer;
    if (delta < -p.max_steer) delta = -p.max_steer;

    e_y_out = e_y;
    e_psi_out = e_psi;
    return delta;
}

static State step_bicycle(const State& s, double delta, double dt, double L) {
    State ns = s;
    ns.x += s.v * std::cos(s.psi) * dt;
    ns.y += s.v * std::sin(s.psi) * dt;
    ns.psi = wrap_angle(s.psi + s.v / L * std::tan(delta) * dt);
    return ns;
}

int main() {
    std::vector<double> xr, yr;
    make_s_path(xr, yr);

    Params p;
    const double dt = 0.02;
    const double T  = 25.0;
    const int steps = static_cast<int>(T / dt);

    State s;
    s.x = -2.0;
    s.y = 3.5;
    s.psi = -10.0 * M_PI / 180.0;
    s.v = 6.0;

    double sum_sq = 0.0;
    for (int k = 0; k < steps; ++k) {
        double e_y = 0.0, e_psi = 0.0;
        double delta = stanley_control(s, xr, yr, p, e_y, e_psi);
        s = step_bicycle(s, delta, dt, p.L);
        sum_sq += e_y * e_y;
    }

    double rms = std::sqrt(sum_sq / steps);
    std::cout << "Done. RMS cross-track error e_y: " << rms << " m\n";
    std::cout << "Final state: x=" << s.x << ", y=" << s.y
              << ", psi=" << s.psi << "\n";
    return 0;
}
