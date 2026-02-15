#include <cmath>
#include <algorithm>

class TwoDofPID {
public:
    TwoDofPID(double Kp, double Ki, double Kd,
              double b, double c,
              double Ts, double N)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd),
          b_(b), c_(c), Ts_(Ts), N_(N),
          integ_(0.0), prev_y_(0.0), prev_r_(0.0), d_state_(0.0)
    {}

    double step(double r, double y) {
        // error signal
        double e = r - y;

        // Proportional term with reference weighting b
        double up = Kp_ * (b_ * r - y);

        // Integral term (backward Euler)
        integ_ += Ki_ * Ts_ * e;
        double ui = integ_;

        // Derivative term with reference weighting c and filtering
        double dr = (r - prev_r_) / Ts_;
        double dy = (y - prev_y_) / Ts_;
        double v = c_ * dr - dy;  // "virtual" derivative input

        // First-order filter on derivative: D(s) = Kd * N / (1 + N / s)
        double alpha = Ts_ * N_;
        d_state_ = (d_state_ + alpha * v) / (1.0 + alpha);
        double ud = Kd_ * d_state_;

        // Update stored values
        prev_r_ = r;
        prev_y_ = y;

        return up + ui + ud;
    }

    void reset() {
        integ_ = 0.0;
        d_state_ = 0.0;
        prev_y_ = 0.0;
        prev_r_ = 0.0;
    }

private:
    double Kp_, Ki_, Kd_;
    double b_, c_;
    double Ts_, N_;
    double integ_;
    double prev_y_, prev_r_;
    double d_state_;
};
