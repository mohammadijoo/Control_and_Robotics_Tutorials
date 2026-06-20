#include <iostream>

struct DCMotorModel {
    double J;
    double b;
    double R_a;
    double L_a;
    double K_t;
    double K_e;

    double theta;
    double omega;
    double i_a;

    DCMotorModel()
        : J(0.01), b(0.1), R_a(1.0), L_a(0.5),
          K_t(0.01), K_e(0.01),
          theta(0.0), omega(0.0), i_a(0.0) {}

    void step(double v_a, double T_load, double dt) {
        double dtheta = omega;
        double domega = (K_t * i_a - b * omega - T_load) / J;
        double di_a = (v_a - R_a * i_a - K_e * omega) / L_a;

        theta += dtheta * dt;
        omega += domega * dt;
        i_a += di_a * dt;
    }
};

int main() {
    DCMotorModel motor;
    double dt = 1e-4;

    for (int k = 0; k < 20000; ++k) {
        double v_a = 24.0;
        double T_load = 0.0;
        motor.step(v_a, T_load, dt);
    }

    std::cout << "Final speed omega = " << motor.omega << std::endl;
    return 0;
}
