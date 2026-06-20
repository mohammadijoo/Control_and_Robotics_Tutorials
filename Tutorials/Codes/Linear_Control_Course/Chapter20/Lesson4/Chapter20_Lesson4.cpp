#pragma once
#include <cmath>

class TwoDofPID
{
public:
    TwoDofPID(double kp, double ki, double kd,
              double b, double c, double dt)
        : kp_(kp), ki_(ki), kd_(kd),
          b_(b), c_(c), dt_(dt),
          integral_(0.0), y_prev_(0.0), r_prev_(0.0)
    {}

    // Update control input for new reference r and measured output y
    double update(double r, double y)
    {
        // Error for integral term
        double e = r - y;
        integral_ += e * dt_;

        // Weighted signals for derivative term
        double v  = c_ * r - y;
        double v_prev = c_ * r_prev_ - y_prev_;
        double dv = (v - v_prev) / dt_;

        // Proportional term uses weight b on reference
        double up = kp_ * (b_ * r - y);
        double ui = ki_ * integral_;
        double ud = kd_ * dv;

        // Store previous values
        y_prev_ = y;
        r_prev_ = r;

        return up + ui + ud;
    }

    void reset()
    {
        integral_ = 0.0;
        y_prev_ = 0.0;
        r_prev_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double b_, c_;
    double dt_;
    double integral_;
    double y_prev_, r_prev_;
};
