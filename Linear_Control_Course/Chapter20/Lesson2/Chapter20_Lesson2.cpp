#include <cmath>

struct TwoDofPid {
    double Kp;
    double Ki;
    double Kd;
    double b;   // proportional weight
    double c;   // derivative weight

    double xi;        // integrator state
    double e_d_prev;  // previous derivative pseudo-error

    TwoDofPid(double Kp_, double Ki_, double Kd_,
              double b_, double c_)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_),
          b(b_), c(c_), xi(0.0), e_d_prev(0.0) {}

    double step(double r, double y, double dt) {
        // 2-DOF pseudo-errors
        double e_p = b * r - y;
        double e_i = r - y;
        double e_d = c * r - y;

        // Integrator
        xi = xi + e_i * dt;

        // Derivative
        double de_d = (e_d - e_d_prev) / dt;
        e_d_prev = e_d;

        // Control output
        double u = Kp * e_p + Ki * xi + Kd * de_d;
        return u;
    }
};

int main() {
    // Example: use the controller in a simple loop
    TwoDofPid pid(2.0, 5.0, 0.1, 0.6, 0.0);
    const double dt = 0.001;
    const int n_steps = 5000;

    double y = 0.0;
    double u = 0.0;
    double r = 1.0;  // unit step

    for (int k = 0; k != n_steps; ++k) {
        u = pid.step(r, y, dt);

        // Replace this simple plant with your robotic actuator model:
        // Example first-order plant: dy/dt = -(1/T)*y + (1/T)*u
        double T = 0.5;
        double dy = (-(1.0 / T) * y + (1.0 / T) * u) * dt;
        y = y + dy;
    }

    return 0;
}
