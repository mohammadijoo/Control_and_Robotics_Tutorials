#include <cmath>

class FilteredDerivative
{
public:
    FilteredDerivative(double Kd, double N, double dt)
    : Kd_(Kd), N_(N), dt_(dt), d_filt_(0.0), e_prev_(0.0)
    {
        alpha_ = std::exp(-N_ * dt_);
    }

    double update(double e)
    {
        // raw discrete derivative of error
        double d_raw = (e - e_prev_) / dt_;

        // low-pass filter with pole around 1/N
        d_filt_ = alpha_ * d_filt_ + (1.0 - alpha_) * d_raw;

        e_prev_ = e;
        return Kd_ * d_filt_;
    }

private:
    double Kd_, N_, dt_;
    double alpha_;
    double d_filt_;
    double e_prev_;
};

// Example usage in a joint controller loop:
//
// FilteredDerivative d_term(2.0, 20.0, dt);
// double u = Kp * e + d_term.update(e);
