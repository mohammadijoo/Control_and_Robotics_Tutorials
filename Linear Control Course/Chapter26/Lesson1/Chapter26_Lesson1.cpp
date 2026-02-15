#include <cmath>

struct FirstOrderLowPass {
    double tau;
    double x;

    explicit FirstOrderLowPass(double tau_, double x0 = 0.0)
        : tau(tau_), x(x0) {}

    double update(double u, double dt) {
        // tau * x_dot + x = u
        const double a = -1.0 / tau;
        const double b =  1.0 / tau;
        x += dt * (a * x + b * u);
        return x;
    }
};

// Example usage in a robotics control loop
// (e.g., inside a ROS node using ros::Rate or a real-time timer).
void controlLoopExample() {
    FirstOrderLowPass encoder_filter(0.02); // 20 ms time constant
    double dt = 0.001;                      // 1 kHz loop

    double encoder_meas = 0.0;
    for (int k = 0; k < 10000; ++k) {
        // Read noisy encoder measurement from hardware / middleware
        // encoder_meas = readEncoderJointPosition();
        double y_filt = encoder_filter.update(encoder_meas, dt);
        // Use y_filt in the feedback law instead of raw encoder_meas
        // tau_c = computeControlTorque(y_filt);
    }
}

// In a ROS-based robot, such a filter integrates with ros_control or custom
// controllers by wrapping the update call inside the controller::update() method.
