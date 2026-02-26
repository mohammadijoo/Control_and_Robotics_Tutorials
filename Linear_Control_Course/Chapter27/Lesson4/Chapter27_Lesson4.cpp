#include <cmath>

// Simple cubic point-to-point trajectory generator.
// Designed to run inside a real-time control loop, e.g. a ROS "joint_trajectory" node.
class CubicTrajectory {
public:
    CubicTrajectory(double q_final, double T_move)
        : q_f_(q_final),
          T_(T_move)
    {}

    // Position profile q(t) for 0 <= t <= T.
    double position(double t) const {
        if (t <= 0.0) return 0.0;
        if (t >= T_) return q_f_;
        double s = t / T_;
        return 3.0 * q_f_ * s * s - 2.0 * q_f_ * s * s * s;
    }

    // Velocity profile dq/dt (optional use for feedforward).
    double velocity(double t) const {
        if (t <= 0.0 || t >= T_) return 0.0;
        double s = t / T_;
        // dq/dt = (6*q_f/T^2)*t - (6*q_f/T^3)*t^2
        return (6.0 * q_f_ / (T_ * T_)) * t
             - (6.0 * q_f_ / (T_ * T_ * T_)) * t * t;
    }

private:
    double q_f_;
    double T_;
};

// Example usage inside a control loop (e.g. in ROS):
//
// CubicTrajectory traj(1.0, 1.0);
// double t = 0.0;
// double dt = 0.001;
// while (ros::ok()) {
//     double q_ref = traj.position(t);
//     // send q_ref to joint PID controller as setpoint
//     t += dt;
// }
