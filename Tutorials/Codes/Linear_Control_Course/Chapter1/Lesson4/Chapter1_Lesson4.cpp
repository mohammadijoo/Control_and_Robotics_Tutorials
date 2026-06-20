#include <iostream>
#include <cmath>

// In a robotic system, this logic could be embedded in a real-time loop,
// possibly inside a ROS control node using roscpp.

int main() {
    double a = 1.0;
    double b = 1.0;
    double k = 5.0;
    double r = 1.0;

    double dt = 0.001;
    double t_final = 5.0;
    int n_steps = static_cast<int>(t_final / dt);

    double x = 0.0;
    double x_ss = 0.0;
    double max_u = 0.0;

    for (int i = 0; i < n_steps; ++i) {
        double e = r - x;
        double u = k * e;
        double dx = -(a + b * k) * x + b * k * r;
        x += dt * dx;

        if (std::fabs(u) > max_u) {
            max_u = std::fabs(u);
        }

        x_ss = x; // last value approximates steady-state
    }

    double e_ss = r - x_ss;
    std::cout << "Approx steady-state output y_ss = " << x_ss << std::endl;
    std::cout << "Approx steady-state error e_ss = " << e_ss << std::endl;
    std::cout << "Peak control effort max|u|    = " << max_u << std::endl;

    return 0;
}
