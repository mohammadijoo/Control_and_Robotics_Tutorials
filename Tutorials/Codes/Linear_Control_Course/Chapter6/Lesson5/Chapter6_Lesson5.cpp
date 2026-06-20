#include <iostream>
#include <vector>

struct SecondOrderSystem {
    double wn;    // natural frequency
    double zeta;  // damping ratio
    double k;     // DC gain

    // State: x1 = y, x2 = y_dot
    double x1;
    double x2;

    SecondOrderSystem(double wn_, double zeta_, double k_)
        : wn(wn_), zeta(zeta_), k(k_), x1(0.0), x2(0.0) {}

    // Compute derivatives for unit-step input u(t) = 1
    void derivatives(double& dx1, double& dx2) const {
        double u = 1.0;
        dx1 = x2;
        // y_ddot + 2*zeta*wn*y_dot + wn^2*y = wn^2*k*u
        dx2 = wn * wn * (k * u - x1) - 2.0 * zeta * wn * x2;
    }
};

int main() {
    double wn   = 5.0;
    double zeta = 0.4;
    double alpha = 50.0;
    double k = 1.0 / alpha;  // match DC gain of third-order system

    SecondOrderSystem sys(wn, zeta, k);

    double dt = 0.0005;
    double t_end = 4.0;
    int steps = static_cast<int>(t_end / dt);

    std::vector<double> t_vals;
    std::vector<double> y_vals;
    t_vals.reserve(steps + 1);
    y_vals.reserve(steps + 1);

    double t = 0.0;
    for (int k_step = 0; k_step <= steps; ++k_step) {
        t_vals.push_back(t);
        y_vals.push_back(sys.x1);

        double dx1, dx2;
        sys.derivatives(dx1, dx2);

        // Explicit Euler integration
        sys.x1 += dt * dx1;
        sys.x2 += dt * dx2;

        t += dt;
    }

    // Print a few samples (in practice, log to file or plot offline)
    for (int i = 0; i < t_vals.size(); i += steps / 10) {
        std::cout << "t = " << t_vals[i]
                  << "  y = " << y_vals[i] << std::endl;
    }

    return 0;
}
