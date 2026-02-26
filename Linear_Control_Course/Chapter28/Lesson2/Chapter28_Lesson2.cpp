#include <iostream>
#include <vector>
#include <cmath>

struct State {
    double x1; // position y
    double x2; // velocity dy/dt
};

State deriv(const State& x, double t, double omega_n, double zeta, double u) {
    (void)t; // t not used in this time-invariant example
    State dx;
    dx.x1 = x.x2;
    dx.x2 = -2.0 * zeta * omega_n * x.x2
            - omega_n * omega_n * x.x1
            + omega_n * omega_n * u;
    return dx;
}

int main() {
    double omega_n = 20.0;
    double zeta    = 0.4;
    double h       = 0.001;   // step size [s]
    double t_final = 4.0;     // final time [s]

    int n_steps = static_cast<int>(t_final / h);
    std::vector<double> t(n_steps + 1);
    std::vector<double> y(n_steps + 1);

    State x{0.0, 0.0}; // initial conditions y(0) = 0, dy/dt(0) = 0

    for (int k = 0; k <= n_steps; ++k) {
        t[k] = k * h;
        y[k] = x.x1;

        double tk = t[k];
        double u  = 1.0; // unit-step input

        State k1 = deriv(x, tk,          omega_n, zeta, u);
        State k2 = deriv({x.x1 + 0.5*h*k1.x1, x.x2 + 0.5*h*k1.x2},
                         tk + 0.5*h, omega_n, zeta, u);
        State k3 = deriv({x.x1 + 0.5*h*k2.x1, x.x2 + 0.5*h*k2.x2},
                         tk + 0.5*h, omega_n, zeta, u);
        State k4 = deriv({x.x1 + h*k3.x1, x.x2 + h*k3.x2},
                         tk + h, omega_n, zeta, u);

        x.x1 += (h / 6.0) * (k1.x1 + 2.0*k2.x1 + 2.0*k3.x1 + k4.x1);
        x.x2 += (h / 6.0) * (k1.x2 + 2.0*k2.x2 + 2.0*k3.x2 + k4.x2);
    }

    // Export y[k] and t[k] to file or plot (e.g., using gnuplot or Python)
    std::cout << "# t  y\n";
    for (int k = 0; k <= n_steps; ++k) {
        std::cout << t[k] << " " << y[k] << "\n";
    }
    return 0;
}
