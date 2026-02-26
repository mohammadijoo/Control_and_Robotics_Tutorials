#include <iostream>
#include <vector>

struct FirstOrderPlant {
    double a; // damping
    double b; // control effectiveness
    double y; // state (output)

    FirstOrderPlant(double a_, double b_, double y0)
        : a(a_), b(b_), y(y0) {}

    void step(double u, double dt) {
        // y_dot = -a y + b u
        double y_dot = -a * y + b * u;
        y += dt * y_dot;
    }
};

int main() {
    double a = 1.0;
    double b = 1.0;
    double K = 4.0;
    double r0 = 1.0;

    double dt = 0.001;
    double T  = 3.0;
    int N = static_cast<int>(T / dt);

    FirstOrderPlant plant(a, b, 0.0);

    std::vector<double> t;
    std::vector<double> y;
    t.reserve(N + 1);
    y.reserve(N + 1);

    double time = 0.0;
    for (int k = 0; k <= N; ++k) {
        t.push_back(time);
        y.push_back(plant.y);

        // Feedback control
        double y_m = plant.y;      // ideal measurement
        double e   = r0 - y_m;     // error
        double u   = K * e;        // proportional control

        plant.step(u, dt);
        time += dt;
    }

    // Print a few samples
    for (int k = 0; k <= N; k += N / 10) {
        std::cout << "t = " << t[k]
                  << ", y = " << y[k] << std::endl;
    }

    // In a ROS-based robot controller, this simulation loop
    // would be replaced by a real-time loop reading from
    // hardware interfaces and publishing actuator commands.
    return 0;
}
