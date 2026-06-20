#include <iostream>
#include <cmath>

int main() {
    double alpha = 0.9;   // coupling strength
    double k = 2.0;       // proportional gain
    double T = 10.0;      // simulation horizon (s)
    double dt = 1e-3;     // time step
    int steps = static_cast<int>(T / dt);

    // State x = [x1; x2], outputs y = x (C = I)
    double x1 = 0.0, x2 = 0.0;

    // Reference: step in r1, r2 = 0
    double r1 = 1.0, r2 = 0.0;

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;

        // Outputs
        double y1 = x1;
        double y2 = x2;

        // Decentralized proportional control u = K (r - y)
        double e1 = r1 - y1;
        double e2 = r2 - y2;
        double u1 = k * e1;
        double u2 = k * e2;

        // State derivatives: xdot = A x + B u
        double x1dot = -x1 + 1.0 * u1 + alpha * u2;
        double x2dot = -x2 + alpha * u1 + 1.0 * u2;

        // Forward Euler step
        x1 += dt * x1dot;
        x2 += dt * x2dot;

        // Print a coarse sample for plotting later
        if (i % 100 == 0) {
            std::cout << t << " " << y1 << " " << y2 << "\n";
        }
    }

    return 0;
}
