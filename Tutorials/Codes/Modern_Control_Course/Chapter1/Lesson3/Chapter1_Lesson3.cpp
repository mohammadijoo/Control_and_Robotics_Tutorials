#include <iostream>
#include <vector>

int main() {
    // Physical parameters
    double m = 1.0;
    double c = 0.4;
    double k = 4.0;

    // Time settings
    double t0 = 0.0;
    double tf = 10.0;
    double h  = 0.001;
    int N = static_cast<int>((tf - t0) / h);

    // State vector x = [x1; x2] = [y; y_dot]
    double x1 = 1.0; // y(0)
    double x2 = 0.0; // y_dot(0)
    double u  = 0.0; // zero-input internal dynamics

    for (int k = 0; k <= N; ++k) {
        double t = t0 + k * h;

        // Print or store state
        if (k % 1000 == 0) {
            std::cout << t << " " << x1 << " " << x2 << std::endl;
        }

        // Compute x_dot = A x + B u for mass-spring-damper
        double x1_dot = x2;
        double x2_dot = -(k / m) * x1 - (c / m) * x2 + (1.0 / m) * u;

        // Euler step
        x1 += h * x1_dot;
        x2 += h * x2_dot;
    }

    return 0;
}
      
