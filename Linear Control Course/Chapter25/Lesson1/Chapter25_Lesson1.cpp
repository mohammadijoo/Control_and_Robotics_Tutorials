#include <iostream>

int main() {
    const double dt = 0.001;      // sampling time [s]
    const int N = 5000;           // simulation steps

    // Controller gains
    const double k1_p = 20.0;     // inner (velocity) P gain
    const double k2_p = 5.0;      // outer (position) P gain

    // Simple first-order plant model for velocity
    double v = 0.0;               // velocity state
    const double a = -50.0;       // pole coefficient
    const double b = 50.0;        // input gain

    double x = 0.0;               // position (integral of velocity)

    double r_pos = 1.0;           // desired position

    for (int k = 0; k < N; ++k) {
        // Outer loop: position control
        double e_pos = r_pos - x;
        double v_ref = k2_p * e_pos;     // desired velocity

        // Inner loop: velocity control
        double e_vel = v_ref - v;
        double u = k1_p * e_vel;        // actuator command (e.g., torque)

        // Plant update (Euler discretization)
        double dv = a * v + b * u;
        v += dv * dt;
        x += v * dt;

        if (k % 1000 == 0) {
            std::cout << "t = " << k * dt
                      << "  x = " << x
                      << "  v = " << v
                      << std::endl;
        }
    }
    return 0;
}
