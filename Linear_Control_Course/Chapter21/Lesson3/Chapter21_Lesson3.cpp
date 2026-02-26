#include <cmath>

class PIController {
public:
    PIController(double Kp, double Ki, double u_min, double u_max)
        : Kp_(Kp), Ki_(Ki), u_min_(u_min), u_max_(u_max),
          integral_(0.0) {}

    double update(double ref, double y, double dt) {
        double e = ref - y;

        // Discrete-time integrator: integral += e * dt
        integral_ += e * dt;

        // Simple anti-windup by clamping the integral term
        double u_int = Ki_ * integral_;
        if (u_int > u_max_) {
            u_int = u_max_;
            integral_ = u_max_ / Ki_;
        } else if (u_int < u_min_) {
            u_int = u_min_;
            integral_ = u_min_ / Ki_;
        }

        double u = Kp_ * e + u_int;

        // Saturate final control signal as well
        if (u > u_max_) u = u_max_;
        if (u < u_min_) u = u_min_;

        return u;
    }

private:
    double Kp_;
    double Ki_;
    double u_min_;
    double u_max_;
    double integral_;
};

// In a robotics control loop (pseudo-code):
// PIController joint_pi(5.0, 10.0, -12.0, 12.0);
// while (robot_running) {
//     double dt = computeDt();
//     double q_ref = getJointRef();
//     double q_meas = getJointPosition();
//     double u = joint_pi.update(q_ref, q_meas, dt);
//     sendTorqueToActuator(u);
// }
