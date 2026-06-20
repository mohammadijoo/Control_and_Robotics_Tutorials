// Chapter18_Lesson2.cpp
// Lagrangian cart-pendulum simulation (generalized coordinates q = [x, theta])
// Standalone C++17 code with RK4 integration and CSV output.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <array>
#include <string>

struct Params {
    double M{1.0};
    double m{0.25};
    double l{0.5};
    double J{0.02};
    double g{9.81};
    double k{8.0};
    double c{0.35};
    double b{0.05};
};

using State = std::array<double, 4>; // [x, theta, xdot, thetadot]

double control_input(double t) {
    return (t >= 0.5 && t <= 2.5) ? 2.0 : 0.0;
}

State rhs(double t, const State& s, const Params& p) {
    const double x = s[0];
    const double th = s[1];
    const double xd = s[2];
    const double thd = s[3];

    const double M11 = p.M + p.m;
    const double M12 = p.m * p.l * std::cos(th);
    const double M21 = M12;
    const double M22 = p.J + p.m * p.l * p.l;

    const double u = control_input(t);
    const double rhs1 = u - p.c * xd - p.k * x + p.m * p.l * std::sin(th) * thd * thd;
    const double rhs2 = -p.b * thd - p.m * p.g * p.l * std::sin(th);

    const double det = M11 * M22 - M12 * M21;
    const double xdd = (rhs1 * M22 - rhs2 * M12) / det;
    const double thdd = (M11 * rhs2 - M21 * rhs1) / det;

    return {xd, thd, xdd, thdd};
}

State add_scaled(const State& a, const State& b, double scale) {
    return {a[0] + scale * b[0], a[1] + scale * b[1], a[2] + scale * b[2], a[3] + scale * b[3]};
}

double total_energy(const State& s, const Params& p) {
    const double x = s[0];
    const double th = s[1];
    const double xd = s[2];
    const double thd = s[3];

    const double T = 0.5 * (p.M + p.m) * xd * xd
                   + p.m * p.l * std::cos(th) * xd * thd
                   + 0.5 * (p.J + p.m * p.l * p.l) * thd * thd;
    const double V = 0.5 * p.k * x * x + p.m * p.g * p.l * (1.0 - std::cos(th));
    return T + V;
}

int main() {
    Params p;
    State s{0.05, 0.35, 0.0, 0.0};

    const double t0 = 0.0;
    const double tf = 10.0;
    const double h = 0.001;
    const int nSteps = static_cast<int>((tf - t0) / h);

    std::ofstream file("Chapter18_Lesson2_cpp_output.csv");
    file << "t,x,theta,xdot,thetadot,energy\n";
    file << std::fixed << std::setprecision(8);

    double t = t0;
    for (int k = 0; k <= nSteps; ++k) {
        file << t << "," << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << "," << total_energy(s, p) << "\n";

        State k1 = rhs(t, s, p);
        State k2 = rhs(t + 0.5 * h, add_scaled(s, k1, 0.5 * h), p);
        State k3 = rhs(t + 0.5 * h, add_scaled(s, k2, 0.5 * h), p);
        State k4 = rhs(t + h, add_scaled(s, k3, h), p);

        for (int i = 0; i < 4; ++i) {
            s[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        t += h;
    }

    file.close();

    std::cout << "Simulation finished. Output saved to Chapter18_Lesson2_cpp_output.csv\n";
    std::cout << "Compile example: g++ -O2 -std=c++17 Chapter18_Lesson2.cpp -o Chapter18_Lesson2\n";
    return 0;
}
