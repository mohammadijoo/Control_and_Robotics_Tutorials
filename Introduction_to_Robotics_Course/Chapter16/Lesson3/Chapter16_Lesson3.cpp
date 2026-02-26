#include <iostream>

int main() {
    // Co-design parameters
    double J  = 0.02;   // kg m^2
    double B  = 0.01;   // N m s/rad
    double N  = 10.0;
    double Kt = 0.05;
    double Ke = 0.05;
    double R  = 2.0;
    double Kp = 32.0;
    double Kd = 1.7;
    double q_ref = 0.5; // rad

    double Beq = B + (N * N) * Kt * Ke / R;
    double Ka  = N * Kt / R;

    double dt    = 0.0005;
    int    steps = 20000; // 10 s of simulation
    double q  = 0.0;
    double qd = 0.0;

    for (int k = 0; k < steps; ++k) {
        double e  = q_ref - q;
        double v  = Kp * e - Kd * qd;
        double tau = Ka * v - Beq * qd;   // J*qdd + Beq*qd = Ka*v
        double qdd = tau / J;

        qd += dt * qdd;
        q  += dt * qd;

        if (k % 4000 == 0) {
            std::cout << (k * dt) << "  " << q << std::endl;
        }
    }
    return 0;
}
      
