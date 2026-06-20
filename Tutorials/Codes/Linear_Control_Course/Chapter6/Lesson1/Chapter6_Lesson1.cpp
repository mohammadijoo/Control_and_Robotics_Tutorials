#include <iostream>
#include <vector>

// Simple numerical simulation of y'' + 2*zeta*wn*y' + wn^2*y = K*wn^2*u
int main() {
    double wn = 4.0;
    double zeta = 0.7;
    double K = 1.0;

    double dt = 0.001;        // integration step (s)
    int N = 5000;             // number of steps (5 seconds)

    double y = 0.0;           // position
    double ydot = 0.0;        // velocity
    double u = 1.0;           // unit step input

    for (int k = 0; k < N; ++k) {
        double t = k * dt;

        // Second-order dynamics
        double yddot =
            -2.0 * zeta * wn * ydot
            - wn * wn * y
            + K * wn * wn * u;

        // Explicit Euler integration
        ydot += dt * yddot;
        y    += dt * ydot;

        // Log a coarse sample for plotting in an external tool
        if (k % 1000 == 0) {
            std::cout << t << " " << y << std::endl;
        }
    }

    return 0;
}

// Note: In a robotics application, this loop would typically run inside a
// real-time control thread, and the state y, ydot would be coupled with
// joint-level feedback, possibly using Eigen for vectorized multi-joint
// models and ros_control for interfacing with the hardware.
