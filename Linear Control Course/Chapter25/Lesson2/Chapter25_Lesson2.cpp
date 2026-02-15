#include <iostream>
#include <vector>

struct InnerPiController {
    double kp;
    double ki;
    double dt;
    double integral;

    InnerPiController(double kp_, double ki_, double dt_)
        : kp(kp_), ki(ki_), dt(dt_), integral(0.0) {}

    double update(double i_ref, double i_meas) {
        double e = i_ref - i_meas;
        integral += e * dt;
        double u = kp * e + ki * integral;
        return u;
    }
};

int main() {
    // RL parameters
    double L = 2e-3;
    double R = 0.5;
    double Ku = 1.0;

    // Same gains as in the Python example (numerical values could be copied)
    double kp = 1.4;   // example value
    double ki = 500.0; // example value
    double dt = 1e-4;  // 10 kHz sampling

    InnerPiController ctrl(kp, ki, dt);

    double i = 0.0;        // current state
    double i_ref = 5.0;    // desired current [A]
    double u = 0.0;        // control voltage

    int steps = 2000;
    for (int k = 0; k < steps; ++k) {
        // Controller update
        u = ctrl.update(i_ref, i);

        // Discrete-time RL model (forward Euler)
        double di_dt = (-R * i + Ku * u) / L;
        i += dt * di_dt;

        if (k % 100 == 0) {
            double t = k * dt;
            std::cout << t << " " << i << std::endl;
        }
    }

    return 0;
}
