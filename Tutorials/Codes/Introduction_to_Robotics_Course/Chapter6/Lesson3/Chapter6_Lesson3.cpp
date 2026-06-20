#include <iostream>
#include <vector>
#include <cmath>

int main() {
    // Parameters
    const double A = 3.0e-3;
    const double beta_e = 1.2e9;
    const double Vt = 6.0e-5;
    const double m = 8.0;
    const double b = 120.0;
    const double Kq = 2.5e-5;
    const double Kp = 1.0e-11;
    const double Ct = 5.0e-12;

    double x = 0.0, xd = 0.0, p = 0.0;
    double dt = 1e-4;
    double T = 1.0;

    auto u_func = [](double t){ return (t >= 0.1) ? 2.0 : 0.0; };
    auto FL_func = [](double t){ return 0.0; };

    for (double t=0.0; t<=T; t+=dt) {
        double u = u_func(t);
        double FL = FL_func(t);

        double q = Kq*u - Kp*p;
        double pd = (beta_e/Vt)*(q - A*xd - Ct*p);
        double xdd = (A*p - b*xd - FL)/m;

        // Euler integration
        x  += xd*dt;
        xd += xdd*dt;
        p  += pd*dt;

        if (static_cast<int>(t/dt) % 1000 == 0) {
            std::cout << t << " " << x << " " << p << std::endl;
        }
    }
    return 0;
}
