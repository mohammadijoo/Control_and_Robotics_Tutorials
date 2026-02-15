#include <iostream>

int main() {
    double tau = 1.0;      // dominant time constant
    double K = 1.0;        // gain
    double dt = 0.001;     // integration step
    double T_end = 8.0;    // simulation horizon
    double x = 0.0;        // state of first-order system
    double u = 1.0;        // unit step input

    // First-order ODE: dx/dt = -(1/tau)*x + (K/tau)*u
    for (double t = 0.0; t <= T_end; t += dt) {
        double dx = -(1.0 / tau) * x + (K / tau) * u;
        x += dt * dx;
        if (static_cast<int>(t * 1000) % 100 == 0) {
            std::cout << t << " " << x << std::endl;
        }
    }
    return 0;
}
