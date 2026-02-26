#include <iostream>
#include <vector>
#include <cmath>

struct FlexParams {
    double J_l{0.5}, J_m{0.05};
    double b_l{0.02}, b_m{0.01};
    double k{150.0}, d{0.5};
    double m{2.0}, ell{0.4}, g{9.81};
    double tau0{1.0};
};

void flex_joint_rhs(const FlexParams& p,
                    double t,
                    const std::vector<double>& x,
                    std::vector<double>& xdot)
{
    double q     = x[0];
    double qdot  = x[1];
    double th    = x[2];
    double thdot = x[3];

    double spring = p.k * (th - q);
    double damper = p.d * (thdot - qdot);
    double tau_m  = p.tau0; // step input

    double qddot  = (spring + damper - p.m * p.g * p.ell * std::sin(q) - p.b_l * qdot) / p.J_l;
    double thddot = (tau_m - spring - damper - p.b_m * thdot) / p.J_m;

    xdot[0] = qdot;
    xdot[1] = qddot;
    xdot[2] = thdot;
    xdot[3] = thddot;
}

int main() {
    FlexParams p;
    double t  = 0.0;
    double dt = 0.0005;
    double T  = 2.0;

    std::vector<double> x(4, 0.0);    // state
    std::vector<double> xdot(4, 0.0); // derivative

    while (t <= T) {
        flex_joint_rhs(p, t, x, xdot);
        // simple explicit Euler integration
        for (int i = 0; i < 4; ++i) {
            x[i] += dt * xdot[i];
        }
        t += dt;
        std::cout << t << " " << x[0] << " " << x[2] << std::endl;
    }
    return 0;
}
      
