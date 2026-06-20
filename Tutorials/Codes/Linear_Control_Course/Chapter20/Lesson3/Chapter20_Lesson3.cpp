#include <cmath>

struct LowPassPrefilter {
    double tau_f;  // continuous-time time constant
    double Ts;     // sampling period
    double y;      // internal state (filtered reference)

    LowPassPrefilter(double tau_f_, double Ts_)
        : tau_f(tau_f_), Ts(Ts_), y(0.0) {}

    // Update with new raw reference r_k, return filtered reference y_k
    double update(double r_k) {
        double alpha = Ts / tau_f;          // discretization gain
        y += alpha * (r_k - y);             // Euler integration
        return y;
    }
};

// Example usage inside a robot joint control loop (pseudo-code):
//
// LowPassPrefilter r_filter(0.1, 0.001);  // tau_f = 0.1 s, Ts = 1 ms
//
// while (true) {
//     double r_raw = read_joint_command();      // desired joint position
//     double r_shaped = r_filter.update(r_raw); // shaped reference
//
//     double y_meas = read_joint_position();    // actual position
//     double u = pid_controller(r_shaped, y_meas);
//     write_actuator_command(u);
// }
