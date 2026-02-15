#include <cmath>
#include <chrono>
#include <thread>

struct JointState {
    double theta;     // rad
    double theta_dot; // rad/s
};

JointState read_joint_state();     // hardware-specific
void write_joint_torque(double tau_cmd); // hardware-specific

int main() {
    const double J = 0.5;
    const double B = 0.1;
    const double omega_n = 4.0;
    const double zeta = 0.7;

    const double Kp = J * omega_n * omega_n;
    const double Kd = 2.0 * zeta * omega_n * J - B;
    const double tau_max = 10.0;
    const double Ts = 0.005; // 5 ms

    auto t0 = std::chrono::steady_clock::now();

    while (true) {
        auto t_now = std::chrono::steady_clock::now();
        double t =
            std::chrono::duration_cast<std::chrono::duration<double> >(
                t_now - t0).count();

        JointState js = read_joint_state();

        // sinusoidal reference
        double theta_d = 0.4 * std::sin(0.5 * t);
        double theta_d_dot = 0.4 * 0.5 * std::cos(0.5 * t);

        double e = theta_d - js.theta;
        double e_dot = theta_d_dot - js.theta_dot;

        double tau_cmd = Kp * e + Kd * e_dot;
        if (tau_cmd > tau_max) tau_cmd = tau_max;
        if (tau_cmd < -tau_max) tau_cmd = -tau_max;

        write_joint_torque(tau_cmd);
        std::this_thread::sleep_for(
            std::chrono::duration<double>(Ts));
    }
    return 0;
}
      
