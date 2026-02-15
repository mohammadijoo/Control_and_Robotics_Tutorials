#include <iostream>

int main() {
    // Plant parameters
    double k = 1.0;
    double tau = 0.3;

    // Controller parameters
    double Kp = 20.0;
    double dt = 0.001;
    double t_final = 2.0;

    // Reference (desired joint position)
    double r = 1.0;

    // State and actuator
    double y = 0.0;      // joint position
    double u = 0.0;      // torque command
    double umax = 50.0;  // saturation

    for (double t = 0.0; t < t_final; t += dt) {
        // Proportional control law
        double e = r - y;
        u = Kp * e;

        // Saturation
        if (u > umax)  u = umax;
        if (u < -umax) u = -umax;

        // First-order joint dynamics (very simplified)
        double dy = (-1.0 / tau) * y + (k / tau) * u;
        y += dy * dt;

        // In a real robot, here we would write u to a motor driver
        // and read y from an encoder instead of simulating.
    }

    std::cout << "Final joint position y = " << y << std::endl;
    return 0;
}
