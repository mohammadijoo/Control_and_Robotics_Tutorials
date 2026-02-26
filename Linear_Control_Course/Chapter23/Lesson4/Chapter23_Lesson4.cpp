#include <iostream>
#include <vector>
#include <random>

// Simple first-order continuous-time model:
//   G(s) = K / (tau s + 1)
// Discretized with forward Euler for simulation.

struct FirstOrderPlant {
    double K;
    double tau;
    double y;    // state (output)
    double Ts;   // sampling period

    FirstOrderPlant(double K_, double tau_, double Ts_)
        : K(K_), tau(tau_), y(0.0), Ts(Ts_) {}

    double step(double u) {
        // Continuous-time: tau * dy/dt = -y + K * u
        // Forward Euler: y[k+1] = y[k] + Ts/tau * (-y[k] + K*u[k])
        double dy = (-y + K * u) * (Ts / tau);
        y += dy;
        return y;
    }
};

int main() {
    double K0 = 10.0;
    double tau0 = 0.05;
    double alpha = 0.2; // 20% gain uncertainty
    double beta  = 0.1; // 10% tau uncertainty
    double Ts    = 0.001;
    int N        = 2000;   // simulation steps

    std::mt19937 gen(1234);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);

    // Nominal plant
    FirstOrderPlant plant_nom(K0, tau0, Ts);

    // Example uncertain plant
    double K_unc = K0 * (1.0 + alpha * uni(gen));
    double tau_unc = tau0 * (1.0 + beta * uni(gen));
    FirstOrderPlant plant_unc(K_unc, tau_unc, Ts);

    std::vector<double> y_nom(N), y_unc(N);

    double u = 1.0; // unit step
    for (int k = 0; k < N; ++k) {
        y_nom[k] = plant_nom.step(u);
        y_unc[k] = plant_unc.step(u);
    }

    // Print a few samples for comparison
    for (int k = 0; k < N; k += 200) {
        std::cout << "k=" << k
                  << "  y_nom=" << y_nom[k]
                  << "  y_unc=" << y_unc[k] << std::endl;
    }

    return 0;
}
