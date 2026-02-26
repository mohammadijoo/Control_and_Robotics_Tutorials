
#include <iostream>
#include <vector>

int main() {
    double Kp = 5.0;
    double tau = 0.2;
    double Ts = 0.001;
    double T_end = 1.0;
    int n_steps = static_cast<int>(T_end / Ts);

    std::vector<double> t(n_steps + 1);
    std::vector<double> r(n_steps + 1, 1.0); // step
    std::vector<double> y(n_steps + 1, 0.0);
    std::vector<double> u(n_steps + 1, 0.0);

    for (int k = 0; k <= n_steps; ++k) {
        t[k] = k * Ts;
    }

    for (int k = 0; k < n_steps; ++k) {
        double e = r[k] - y[k];
        u[k] = Kp * e;
        double dy = (-y[k] + u[k]) / tau;
        y[k + 1] = y[k] + Ts * dy;
    }

    // Print a few samples
    for (int k = 0; k < n_steps; k += n_steps / 10) {
        std::cout << "t=" << t[k]
                  << "  r=" << r[k]
                  << "  y=" << y[k] << std::endl;
    }
    return 0;
}
