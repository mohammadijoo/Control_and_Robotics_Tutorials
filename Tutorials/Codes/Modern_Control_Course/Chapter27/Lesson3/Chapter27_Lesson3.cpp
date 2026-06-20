// Chapter27_Lesson3.cpp
// From-scratch RK4 simulation of a state-feedback servo with one integrator.
// Compile: g++ -std=c++17 Chapter27_Lesson3.cpp -O2 -o Chapter27_Lesson3

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct State {
    double x1;
    double x2;
    double eta;
};

State add_scaled(const State& a, const State& b, double h) {
    return {a.x1 + h * b.x1, a.x2 + h * b.x2, a.eta + h * b.eta};
}

State rhs(const State& s, double reference) {
    // Plant: x1_dot = x2
    //        x2_dot = -2*x1 - 0.4*x2 + u
    // Output: y = x1
    // Internal model: eta_dot = reference - y
    // Closed-loop gain for desired poles -2 +/- 2i and -5:
    // u = -26*x1 - 8.6*x2 + 40*eta
    double y = s.x1;
    double u = -26.0 * s.x1 - 8.6 * s.x2 + 40.0 * s.eta;

    State ds;
    ds.x1 = s.x2;
    ds.x2 = -2.0 * s.x1 - 0.4 * s.x2 + u;
    ds.eta = reference - y;
    return ds;
}

State rk4_step(const State& s, double h, double reference) {
    State k1 = rhs(s, reference);
    State k2 = rhs(add_scaled(s, k1, 0.5 * h), reference);
    State k3 = rhs(add_scaled(s, k2, 0.5 * h), reference);
    State k4 = rhs(add_scaled(s, k3, h), reference);

    State out;
    out.x1 = s.x1 + h * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1) / 6.0;
    out.x2 = s.x2 + h * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2) / 6.0;
    out.eta = s.eta + h * (k1.eta + 2.0 * k2.eta + 2.0 * k3.eta + k4.eta) / 6.0;
    return out;
}

int main() {
    const double reference = 1.0;
    const double h = 0.001;
    const double tf = 8.0;
    const int steps = static_cast<int>(tf / h);

    State s{0.0, 0.0, 0.0};
    double peak_abs_u = 0.0;

    for (int k = 0; k < steps; ++k) {
        double u = -26.0 * s.x1 - 8.6 * s.x2 + 40.0 * s.eta;
        peak_abs_u = std::max(peak_abs_u, std::abs(u));
        s = rk4_step(s, h, reference);
    }

    double y = s.x1;
    double error = reference - y;

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "Final output y(T) = " << y << "\n";
    std::cout << "Final tracking error e(T) = " << error << "\n";
    std::cout << "Final integrator state eta(T) = " << s.eta << "\n";
    std::cout << "Peak absolute control = " << peak_abs_u << "\n";
    return 0;
}
