#include <iostream>

struct PDController {
    double Kp;
    double Kd;
    double dt;
    double prev_error;

    PDController(double Kp_, double Kd_, double dt_)
        : Kp(Kp_), Kd(Kd_), dt(dt_), prev_error(0.0) {}

    double compute(double desired, double measured) {
        double error = desired - measured;
        double dedt = (error - prev_error) / dt;  // discrete derivative
        prev_error = error;
        return Kp * error + Kd * dedt;
    }
};

int main() {
    double Kp = 16.0;
    double Kd = 4.0;
    double dt = 0.001; // 1 kHz control loop

    PDController pd(Kp, Kd, dt);

    double y = 0.0;      // joint position
    double ydot = 0.0;   // joint velocity
    double desired = 1.0;
    double M = 1.0, B = 0.5, K = 4.0;

    for (int k = 0; k < 10000; ++k) {
        double t = k * dt;
        double u = pd.compute(desired, y);

        // Simple forward-Euler integration of mass-spring-damper
        double yddot = (u - B * ydot - K * y) / M;
        ydot += yddot * dt;
        y   += ydot * dt;

        if (k % 1000 == 0) {
            std::cout << "t=" << t
                      << " y=" << y
                      << " u=" << u << std::endl;
        }
    }

    return 0;
}
