#include <iostream>
#include <vector>
#include <cmath>

double sat(double v, double umin, double umax) {
    if (v < umin) return umin;
    if (v > umax) return umax;
    return v;
}

int main() {
    const double zeta = 0.3;
    const double wn   = 2.0;

    double t0 = 0.0;
    double tf = 10.0;
    double dt = 0.001;

    double y    = 0.0;
    double ydot = 0.0;

    auto u_cmd = [](double t) {
        return 2.0; // step command that saturates
    };

    std::vector<double> t_vec;
    std::vector<double> y_vec;

    for (double t = t0; t <= tf; t += dt) {
        // time-varying parameters
        double a1 = 2*zeta*wn * (1.0 + 0.5*std::sin(0.5*t));
        double a0 = wn*wn * (1.0 + 0.3*std::cos(0.2*t));

        double u_phys = sat(u_cmd(t), -1.0, 1.0);

        double yddot = -a1*ydot - a0*y + wn*wn*u_phys;

        // Euler integration
        y    += dt*ydot;
        ydot += dt*yddot;

        t_vec.push_back(t);
        y_vec.push_back(y);
    }

    // For brevity, print only a few samples
    for (std::size_t k = 0; k < t_vec.size(); k += 1000) {
        std::cout << t_vec[k] << " " << y_vec[k] << "\n";
    }
    return 0;
}
      
