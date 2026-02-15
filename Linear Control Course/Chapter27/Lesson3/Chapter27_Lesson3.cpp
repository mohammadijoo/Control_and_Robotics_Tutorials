#include <iostream>
#include <vector>

int main() {
    // Physical parameters
    double J = 0.01;   // inertia [kg m^2]
    double b = 0.1;    // viscous friction [N m s/rad]

    // PI controller gains (same as Python example)
    double Kp = 20.0;
    double Ki = 50.0;

    // Simulation settings
    double Ts = 0.001;       // sampling period [s]
    double Tfinal = 5.0;     // total time [s]
    int N = static_cast<int>(Tfinal / Ts);

    // State variables: position theta, velocity omega, integral of error
    double theta = 0.0;
    double omega = 0.0;
    double e_int = 0.0;

    // Reference position (regulation)
    double theta_ref = 0.0;

    // Disturbance: step load torque
    double D0 = 1.0;  // [N m]

    for (int k = 0; k < N; ++k) {
        double t = k * Ts;

        // Step disturbance applied for t >= 0
        double d = D0;

        // Error and integral
        double e = theta_ref - theta;
        e_int += e * Ts;

        // Control torque
        double u = Kp * e + Ki * e_int;

        // Joint dynamics: J * theta_ddot + b * theta_dot = u - d
        double theta_ddot = (u - d - b * omega) / J;

        // Euler integration
        omega += Ts * theta_ddot;
        theta += Ts * omega;

        if (k % 1000 == 0) {
            std::cout << "t=" << t
                      << "  theta=" << theta
                      << "  omega=" << omega
                      << std::endl;
        }
    }

    std::cout << "Final position (steady-state error) [rad]: "
              << theta << std::endl;

    return 0;
}
