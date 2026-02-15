#include <iostream>
#include <vector>

int main() {
    // Mechanical parameters (example)
    double J_eq = 0.01;
    double b_eq = 0.001;
    double K_t  = 0.1;
    double K_c  = 2.0;
    double N    = 50.0;

    double Kv    = (K_t * K_c) / (N * b_eq);
    double tau_m = J_eq / b_eq;

    // Discrete-time parameters
    double dt   = 0.0005;     // sampling period [s]
    int    Nsim = 4000;       // number of samples

    // PID gains (tuned by hand or from design rules)
    double Kp = 50.0;
    double Ki = 500.0;
    double Kd = 0.001;

    // State: x1 = theta, x2 = theta_dot
    double x1 = 0.0;
    double x2 = 0.0;

    double r     = 0.1;   // step reference [rad]
    double e     = 0.0;
    double e_prev = 0.0;
    double I     = 0.0;

    for (int k = 0; k < Nsim; ++k) {
        double t = k * dt;

        // Position error
        e = r - x1;

        // PID control law (parallel form)
        I += e * dt;
        double D = (e - e_prev) / dt;
        double u = Kp * e + Ki * I + Kd * D;

        // Plant: theta_ddot = -(b_eq/J_eq)*theta_dot + (Kv/J_eq)*u
        double x2_dot = -(b_eq / J_eq) * x2 + (Kv / J_eq) * u;
        double x1_dot = x2;

        // Euler integration
        x2 += x2_dot * dt;
        x1 += x1_dot * dt;

        e_prev = e;

        if (k % 200 == 0) {
            std::cout << "t=" << t
                      << " theta=" << x1
                      << " u=" << u << std::endl;
        }
    }

    return 0;
}
