
#include <iostream>
#include <cmath>

enum class Mode { Impedance, Admittance };

struct InteractionState {
    double x;    // position
    double v;    // velocity
    double x_c;  // commanded position (admittance)
};

void stepInteraction(InteractionState& s,
                     double x_d,
                     double dt,
                     Mode mode)
{
    // Robot and environment parameters
    const double M_r = 5.0;
    const double D_r = 2.0;
    const double K_e = 500.0;
    const double D_e = 10.0;

    const double M_d = 2.0;
    const double D_d = 30.0;
    const double K_d = 200.0;

    // Environment force
    double F_env = K_e * s.x + D_e * s.v;

    double F_u = 0.0;

    if (mode == Mode::Impedance) {
        double e  = x_d - s.x;
        double ed = 0.0 - s.v;

        F_u = K_d * e + D_d * ed;

        double a = (F_u + F_env - D_r * s.v) / M_r;
        s.v += dt * a;
        s.x += dt * s.v;

    } else { // Admittance
        static double v_c = 0.0;  // virtual velocity
        double a_c = (F_env - D_d * v_c - K_d * (s.x_c - x_d)) / M_d;
        v_c += dt * a_c;
        s.x_c += dt * v_c;

        // Inner position servo
        double e_pos = s.x_c - s.x;
        F_u = 1000.0 * e_pos;

        double a = (F_u + F_env - D_r * s.v) / M_r;
        s.v += dt * a;
        s.x += dt * s.v;
    }
}
