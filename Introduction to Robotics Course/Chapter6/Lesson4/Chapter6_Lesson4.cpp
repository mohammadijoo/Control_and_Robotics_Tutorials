#include <iostream>
#include <cmath>

int main() {
    // Parameters
    const double M = 2.0;          // kg
    const double A = 3.0e-4;       // m^2
    const double V0 = 2.0e-5;      // m^3
    const double k = 1.2;
    const double R = 287.0;
    const double T = 293.0;
    const double b = 15.0;
    const double p_atm = 101325.0;
    const double Kq = 1.3e-4;
    const double Kp = 2.0e-9;

    // State
    double x = 0.0, v = 0.0, p = p_atm;

    // Integration
    const double dt = 1e-4;
    const double tf = 0.4;

    for (double t = 0.0; t < tf; t += dt) {
        double V = V0 + A*x;
        double u = (t >= 0.05) ? 1.0 : 0.0;
        double mdot = Kq*u - Kp*(p - p_atm);

        double pdot = (k*R*T/V)*mdot - (k*p/V)*(A*v);
        double F = A*p - b*v;
        double vdot = F/M;

        x += v*dt;
        v += vdot*dt;
        p += pdot*dt;
    }

    std::cout << "Final x = " << x << " m\n";
    std::cout << "Final p = " << p/1000.0 << " kPa\n";
    return 0;
}
