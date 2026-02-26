#include <iostream>

int main() {
    // Thermal parameters
    const double tau_th = 300.0;   // s
    const double K_th   = 5.0;     // K per unit input

    // PI gains (from previous design or manual tuning)
    const double Kp = 0.05;
    const double Ki = 1.0e-4;

    // Discretization
    const double Ts = 1.0;         // sampling period [s]
    const int    N  = 4000;        // number of simulation steps

    // State variables
    double T = 0.0;                // Delta T, deviation from nominal [K]
    double integ = 0.0;            // integral of error
    double u = 0.0;                // control input (Delta u)
    const double r = 1.0;          // 1 K step reference

    for (int k = 0; k < N; ++k) {
        double e = r - T;
        integ += e * Ts;
        u = Kp * e + Ki * integ;

        // Saturation of heater command (0 to 1 in absolute units)
        if (u > 1.0) u = 1.0;
        if (u < 0.0) u = 0.0;

        // Thermal plant update: forward Euler
        double dTdt = (-1.0 / tau_th) * T + (K_th / tau_th) * u;
        T += Ts * dTdt;

        // Log every 100 steps
        if (k % 100 == 0) {
            std::cout << "t = " << k * Ts
                      << " s, T = " << T
                      << " K, u = " << u
                      << std::endl;
        }
    }
    return 0;
}
