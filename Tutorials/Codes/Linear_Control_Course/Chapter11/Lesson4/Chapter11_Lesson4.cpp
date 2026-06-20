#include <algorithm>
#include <cmath>

class PID {
public:
    PID(double Kp, double Ki, double Kd, double dt,
        double u_min = -1e9, double u_max = 1e9)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd),
          dt_(dt), u_min_(u_min), u_max_(u_max),
          integral_(0.0), e_prev_(0.0), first_call_(true) {}

    void reset() {
        integral_ = 0.0;
        e_prev_   = 0.0;
        first_call_ = true;
    }

    double update(double r, double y) {
        double e = r - y;

        // Integral term
        integral_ += e * dt_;

        // Derivative term
        double d = 0.0;
        if (!first_call_) {
            d = (e - e_prev_) / dt_;
        } else {
            first_call_ = false;
        }

        double u = Kp_ * e + Ki_ * integral_ + Kd_ * d;

        // Saturation
        u = std::max(u_min_, std::min(u_max_, u));

        e_prev_ = e;
        return u;
    }

private:
    double Kp_, Ki_, Kd_;
    double dt_;
    double u_min_, u_max_;
    double integral_;
    double e_prev_;
    bool first_call_;
};

/*
 * In ROS, this PID class could be embedded inside a controller plugin,
 * using the ros_control/controller_interface API. The structure here
 * corresponds to the parallel form (Kp, Ki, Kd) used by many robotics
 * libraries such as control_toolbox::Pid.
 */
