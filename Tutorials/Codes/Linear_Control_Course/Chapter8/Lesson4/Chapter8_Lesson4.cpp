#include <iostream>
#include <vector>

// Simple first-order joint model: y_dot = -3 y + 5 u
// Discretized with forward Euler for illustration.

struct PIController {
    double Kp;
    double Ki;
    double integral;

    PIController(double Kp_, double Ki_)
        : Kp(Kp_), Ki(Ki_), integral(0.0) {}

    double update(double r, double y, double dt) {
        double e = r - y;
        integral += e * dt;
        double u = Kp * e + Ki * integral;
        return u;
    }
};

int main() {
    double dt = 0.001;          // 1 ms control period
    double T  = 5.0;            // simulation horizon
    int N = static_cast<int>(T / dt);

    double y = 0.0;             // joint position
    double r = 1.0;             // unit step reference

    PIController pi(1.0, 4.0);  // gains as in the analytic design

    std::vector<double> e_hist;
    e_hist.reserve(N);

    for (int k = 0; k < N; ++k) {
        double t = k * dt;

        // For ramp tracking, replace r = 1.0 with r = t
        double u = pi.update(r, y, dt);

        // Plant integration: y_dot = -3 y + 5 u
        double y_dot = -3.0 * y + 5.0 * u;
        y += dt * y_dot;

        double e = r - y;
        e_hist.push_back(e);
    }

    std::cout << "Approximate steady-state error (last sample): "
              << e_hist.back() << std::endl;

    return 0;
}
