#include <iostream>
#include <vector>

// Simple Euler simulation of process with lag controller.
// This style is typical in low-level robotics controllers implemented in C++
// (for example inside a ROS control loop) where the continuous-time design
// is discretized with a fixed time step.

int main() {
    const double Kp = 2.0;
    const double Tp = 5.0;

    const double Kc = 24.5;
    const double beta = 22.0;
    const double Tl = 25.0;

    const double h = 0.01;        // simulation step (s)
    const double t_end = 50.0;    // total simulation time (s)
    const double r = 1.0;         // unit step reference

    double x = 0.0;   // process state
    double w = 0.0;   // lag filter state
    double y = 0.0;   // process output
    double e = 0.0;   // tracking error
    double u = 0.0;   // control input

    std::size_t steps = static_cast<std::size_t>(t_end / h);

    for (std::size_t k = 0; k < steps; ++k) {
        double t = k * h;

        y = x;
        e = r - y;

        // Lag filter: beta * Tl * dw/dt + w = e
        double dw = (e - w) / (beta * Tl);
        w += h * dw;

        // Control law: u = Kc * ((1 - 1/beta) * w + (1/beta) * e)
        u = Kc * ((1.0 - 1.0 / beta) * w + (1.0 / beta) * e);

        // Process dynamics: dx/dt = -(1/Tp) * x + (Kp/Tp) * u
        double dx = -(1.0 / Tp) * x + (Kp / Tp) * u;
        x += h * dx;

        if (k % 100 == 0) {
            std::cout << t << " " << y << std::endl;
        }
    }

    return 0;
}
