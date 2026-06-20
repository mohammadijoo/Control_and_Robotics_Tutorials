#include <iostream>
#include <fstream>
#include <cmath>

double sat(double u, double u_max) {
    if (u > u_max) return u_max;
    if (u < -u_max) return -u_max;
    return u;
}

int main() {
    const double omega_n = 1.0;
    const double zeta = 0.2;
    const double K = 8.0;
    const double u_max = 2.0;

    const double t_final = 20.0;
    const double dt = 1e-3;
    const int n_steps = static_cast<int>(t_final / dt);

    double x1 = 0.0, x2 = 0.0;
    double t = 0.0;

    std::ofstream fout("saturation_response.csv");
    fout << "t,y,u\n";

    for (int k = 0; k <= n_steps; ++k) {
        double y = x1;
        double r = 1.0;
        double e = r - y;
        double v = K * e;
        double u = sat(v, u_max);

        fout << t << "," << y << "," << u << "\n";

        // plant dynamics
        double dx1 = x2;
        double dx2 = -2.0 * zeta * omega_n * x2
                     - omega_n * omega_n * x1
                     + omega_n * omega_n * u;

        x1 += dt * dx1;
        x2 += dt * dx2;
        t += dt;
    }

    fout.close();
    std::cout << "Simulation data written to saturation_response.csv\n";
    return 0;
}
