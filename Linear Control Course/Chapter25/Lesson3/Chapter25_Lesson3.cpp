#include <iostream>
#include <vector>

int main() {
    // Continuous-time model: x_dot = -x + u, y = x
    // Inner loop: u = k_i * (w - y)
    // Outer loop: w = k_p * e_o + z,  z_dot = k_I * e_o (PI integrator)
    double k_i = 9.0;
    double k_p = 0.5;
    double k_I = 1.0;

    double h = 1e-3;           // simulation step [s]
    double t_end = 5.0;
    int n_steps = static_cast<int>(t_end / h);

    double x = 0.0;            // plant state
    double z = 0.0;            // integral state of outer PI
    double y = 0.0;            // output
    double r = 1.0;            // step reference

    std::vector<double> t_hist, y_hist;
    t_hist.reserve(n_steps);
    y_hist.reserve(n_steps);

    for (int k = 0; k < n_steps; ++k) {
        double t = k * h;

        // Outer loop
        double e_outer = r - y;
        z += h * (k_I * e_outer);   // integrator
        double w = k_p * e_outer + z;

        // Inner loop (P)
        double u = k_i * (w - y);

        // Plant integration: x_dot = -x + u
        double x_dot = -x + u;
        x += h * x_dot;
        y = x;

        t_hist.push_back(t);
        y_hist.push_back(y);
    }

    // Print a few samples
    for (int i = 0; i < 10; ++i) {
        int idx = i * (n_steps / 10);
        std::cout << "t=" << t_hist[idx]
                  << "  y=" << y_hist[idx] << std::endl;
    }
    return 0;
}
