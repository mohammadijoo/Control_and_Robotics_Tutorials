#include <iostream>
#include <vector>

struct PID {
    double Kp;
    double Ki;
    double Kd;
    double dt;

    double integrator;
    double prev_error;

    PID(double Kp_, double Ki_, double Kd_, double dt_)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_), dt(dt_),
          integrator(0.0), prev_error(0.0) {}

    double update(double error) {
        // Trapezoidal integration for I term
        integrator += 0.5 * (error + prev_error) * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integrator + Kd * derivative;
    }
};

// Example: first-order plant x_dot = (-1/T)*x + (K/T)*u
struct FirstOrderPlant {
    double T;
    double K;
    double x;

    FirstOrderPlant(double T_, double K_)
        : T(T_), K(K_), x(0.0) {}

    double step(double u, double dt) {
        double dx = (-x / T) + (K / T) * u;
        x += dt * dx;
        return x; // y = x
    }
};

int main() {
    double dt = 0.001;
    PID lowLevelPID(2.0, 50.0, 0.01, dt);
    FirstOrderPlant plant(0.05, 1.0);

    double r = 1.0;           // reference from higher-level module
    std::vector<double> y_log, u_log;

    double y = 0.0;
    for (int k = 0; k < 20000; ++k) {
        double error = r - y;
        double u = lowLevelPID.update(error);

        // Plant update
        y = plant.step(u, dt);

        y_log.push_back(y);
        u_log.push_back(u);
    }

    std::cout << "Final output y = " << y << std::endl;
    return 0;
}
