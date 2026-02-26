#include <iostream>
#include <cmath>

int main() {
    double K = 5.0;
    double tau = 0.1;
    double U = 1.0;
    double omega = 10.0;

    double dt = 1e-4;
    double t_final = 2.0;
    int steps = static_cast<int>(t_final / dt);

    double y = 0.0;   // output state
    double t = 0.0;

    for (int k = 0; k < steps; ++k) {
        double u = U * std::sin(omega * t);
        double dy = (-y + K * u) / tau;  // tau * dy/dt + y = K u

        // Forward Euler update
        y += dt * dy;
        t += dt;

        if (k % 1000 == 0) {
            std::cout << t << " " << u << " " << y << std::endl;
        }
    }

    return 0;
}
