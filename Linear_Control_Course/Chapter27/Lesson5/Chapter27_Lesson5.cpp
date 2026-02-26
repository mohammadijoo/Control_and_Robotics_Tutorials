#include <iostream>
#include <vector>

int main() {
    // Controller gains
    const double Kp = 3.2;
    const double Ki = 9.0;

    // Simulation parameters
    const double dt = 0.001;
    const double t_final = 8.0;
    const int n_steps = static_cast<int>(t_final / dt) + 1;

    std::vector<double> t(n_steps), y(n_steps), z(n_steps), u(n_steps), d(n_steps);

    for (int k = 0; k < n_steps; ++k) {
        t[k] = k * dt;
    }

    auto reference = [](double time) {
        return 1.0; // unit step
    };

    auto disturbance = [](double time) {
        return (time >= 3.0) ? 0.2 : 0.0;
    };

    for (int k = 0; k < n_steps - 1; ++k) {
        double r_k = reference(t[k]);
        double d_k = disturbance(t[k]);
        double e_k = r_k - y[k];

        // PI control
        u[k] = Kp * e_k + Ki * z[k];

        // State derivatives
        double dz = e_k;
        double dy = -y[k] + u[k] + d_k;

        // Euler integration
        z[k + 1] = z[k] + dt * dz;
        y[k + 1] = y[k] + dt * dy;
        d[k]     = d_k;
    }
    // Final values
    d[n_steps - 1] = disturbance(t[n_steps - 1]);
    u[n_steps - 1] = Kp * (reference(t[n_steps - 1]) - y[n_steps - 1]) + Ki * z[n_steps - 1];

    // In a ROS controller, this loop structure would be embedded in a realtime
    // callback, publishing 'u' to an actuator and reading 'y' from sensors.
    std::cout << "Simulation finished, final output y(T) = " << y.back() << std::endl;
    return 0;
}
