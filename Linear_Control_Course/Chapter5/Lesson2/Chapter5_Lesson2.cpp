#include <iostream>
#include <vector>
#include <cmath>

int main() {
    double K   = 2.0;
    double tau = 0.5;
    double dt  = 0.001;
    double T   = 5.0;

    int N = static_cast<int>(T / dt);
    std::vector<double> t_vec(N), y_vec(N);

    double y = 0.0; // initial condition

    for (int k = 0; k < N; ++k) {
        double t = k * dt;

        // Choose test input:
        double u_step    = 1.0;           // unit step
        double u_ramp    = t;             // unit ramp
        double u_impulse = (k == 0) ? 1.0 / dt : 0.0; // discrete impulse

        double u = u_step; // switch to u_ramp or u_impulse for other tests

        // First-order ODE: tau dy/dt + y = K u
        double dydt = (-y + K * u) / tau;
        y += dt * dydt;

        t_vec[k] = t;
        y_vec[k] = y;
    }

    // Print a few samples
    for (int k = 0; k < N; k += N / 10) {
        std::cout << "t = " << t_vec[k]
                  << ", y = " << y_vec[k] << std::endl;
    }

    // In robotics, this loop can be placed inside a real-time control thread,
    // with u read from sensor-based trajectories or ROS topics.
    return 0;
}
