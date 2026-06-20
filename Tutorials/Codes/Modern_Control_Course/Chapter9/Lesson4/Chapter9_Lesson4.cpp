/*
Chapter9_Lesson4.cpp

Compile:
    g++ -std=c++17 -O2 Chapter9_Lesson4.cpp -o Chapter9_Lesson4

This program simulates:
    A = diag(-1, +1), B = [1, 0]^T, C = [1, 0], D = 0

The input-output transfer function is G(s)=1/(s+1), but the second internal
mode x2_dot = x2 is unstable and hidden from the input-output channel.
*/

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

struct State {
    double x1;
    double x2;
};

State derivative(const State& x, double u) {
    State dx;
    dx.x1 = -x.x1 + u;
    dx.x2 =  x.x2;
    return dx;
}

State add_scaled(const State& x, const State& k, double scale) {
    return State{x.x1 + scale * k.x1, x.x2 + scale * k.x2};
}

State rk4_step(const State& x, double u, double h) {
    State k1 = derivative(x, u);
    State k2 = derivative(add_scaled(x, k1, 0.5 * h), u);
    State k3 = derivative(add_scaled(x, k2, 0.5 * h), u);
    State k4 = derivative(add_scaled(x, k3, h), u);

    return State{
        x.x1 + (h / 6.0) * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1),
        x.x2 + (h / 6.0) * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2)
    };
}

double output(const State& x, double u) {
    (void)u;
    return x.x1;
}

void simulate(const State& x0,
              const std::function<double(double)>& u_fun,
              double t_final,
              double h,
              const std::string& case_name) {
    State x = x0;
    double max_abs_y = 0.0;
    double max_norm_x = std::sqrt(x.x1 * x.x1 + x.x2 * x.x2);
    double y = output(x, u_fun(0.0));

    int n_steps = static_cast<int>(t_final / h);
    for (int k = 0; k < n_steps; ++k) {
        double t = k * h;
        double u = u_fun(t);
        y = output(x, u);
        max_abs_y = std::max(max_abs_y, std::abs(y));
        max_norm_x = std::max(max_norm_x, std::sqrt(x.x1 * x.x1 + x.x2 * x.x2));
        x = rk4_step(x, u, h);
    }

    y = output(x, u_fun(t_final));
    max_abs_y = std::max(max_abs_y, std::abs(y));
    max_norm_x = std::max(max_norm_x, std::sqrt(x.x1 * x.x1 + x.x2 * x.x2));

    std::cout << "\n" << case_name << "\n";
    std::cout << "  final x1 = " << x.x1 << ", final x2 = " << x.x2 << "\n";
    std::cout << "  final y  = " << y << "\n";
    std::cout << "  max |y|  = " << max_abs_y << "\n";
    std::cout << "  max ||x||= " << max_norm_x << "\n";
}

int main() {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "A eigenvalues are -1 and +1.\n";
    std::cout << "Internal stability: unstable because one eigenvalue has positive real part.\n";
    std::cout << "External transfer function from u to y: G(s) = 1/(s + 1), externally stable.\n";

    simulate(State{0.0, 1.0},
             [](double /*t*/) { return 0.0; },
             5.0,
             0.01,
             "Case 1: zero input, hidden unstable initial condition x(0)=[0,1]^T");

    simulate(State{0.0, 0.0},
             [](double /*t*/) { return 1.0; },
             5.0,
             0.01,
             "Case 2: unit step input, zero initial state");

    return 0;
}
