#include <algorithm>
#include <limits>

enum class ZNMode { P, PI, PID };

struct ZNParams {
    double Kp;
    double Ki;
    double Kd;
};

inline ZNParams znClosedLoop(double Ku, double Tu, ZNMode mode) {
    double Kp, Ti, Td;
    if (mode == ZNMode::P) {
        Kp = 0.5 * Ku;
        Ti = std::numeric_limits<double>::infinity();
        Td = 0.0;
    } else if (mode == ZNMode::PI) {
        Kp = 0.45 * Ku;
        Ti = Tu / 1.2;
        Td = 0.0;
    } else { // PID
        Kp = 0.60 * Ku;
        Ti = Tu / 2.0;
        Td = Tu / 8.0;
    }

    double Ki = std::isinf(Ti) ? 0.0 : Kp / Ti;
    double Kd = Kp * Td;
    return {Kp, Ki, Kd};
}

class PID {
public:
    PID(double Kp, double Ki, double Kd, double dt,
        double u_min = -std::numeric_limits<double>::infinity(),
        double u_max =  std::numeric_limits<double>::infinity())
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt),
          u_min_(u_min), u_max_(u_max),
          integral_(0.0), prev_error_(0.0) {}

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

    double step(double error) {
        integral_ += error * dt_;
        double derivative = (error - prev_error_) / dt_;

        double u = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

        if (u > u_max_) {
            u = u_max_;
            integral_ -= error * dt_;
        } else if (u < u_min_) {
            u = u_min_;
            integral_ -= error * dt_;
        }

        prev_error_ = error;
        return u;
    }

private:
    double Kp_, Ki_, Kd_;
    double dt_;
    double u_min_, u_max_;
    double integral_;
    double prev_error_;
};

// Example usage in a joint position controller:
// ZNParams p = znClosedLoop(Ku, Tu, ZNMode::PID);
// PID pid(p.Kp, p.Ki, p.Kd, dt, -10.0, 10.0);
// double torque = pid.step(position_error);
