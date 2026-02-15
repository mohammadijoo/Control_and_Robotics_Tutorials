#include <iostream>
#include <fstream>

int main() {
    // Motor parameters
    double R = 2.0;
    double L = 0.5;
    double J = 0.02;
    double b = 0.002;
    double Kt = 0.1;
    double Ke = 0.1;

    // PI speed controller gains (use values designed in Python, for example)
    double Kp = 1.5;
    double Ki = 50.0;

    // Simulation settings
    double dt = 0.0005;
    double t_end = 1.0;
    int steps = static_cast<int>(t_end / dt);

    // State variables
    double ia = 0.0;      // armature current
    double omega = 0.0;   // speed
    double theta = 0.0;   // position
    double integ_e = 0.0; // integrator state for PI
    double TL = 0.0;      // load torque

    std::ofstream fout("dc_motor_pi_speed.csv");
    fout << "t,omega,theta,ia\n";

    for (int k = 0; k <= steps; ++k) {
        double t = k * dt;

        // Reference speed: unit step
        double omega_ref = (t >= 0.0) ? 1.0 : 0.0;

        // Speed error
        double e = omega_ref - omega;

        // PI controller
        integ_e += e * dt;
        double va = Kp * e + Ki * integ_e;

        // Electrical dynamics: L dia/dt = va - R ia - Ke omega
        double dia = (va - R * ia - Ke * omega) / L;

        // Mechanical dynamics: J domega/dt = Kt ia - b omega - TL
        double domega = (Kt * ia - b * omega - TL) / J;

        // Integrate states
        ia += dia * dt;
        omega += domega * dt;
        theta += omega * dt;

        fout << t << "," << omega << "," << theta << "," << ia << "\n";
    }

    fout.close();
    std::cout << "Simulation complete. Data written to dc_motor_pi_speed.csv\n";
    return 0;
}
