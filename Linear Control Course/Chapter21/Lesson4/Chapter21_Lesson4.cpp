#include <cmath>
#include <vector>

// In practice you would integrate this into a ROS controller node and use Eigen
// for vector operations. Here we show a scalar example.

class LowPassFilter {
public:
    LowPassFilter(double Ts, double w_h)
    : Ts_(Ts), w_h_(w_h)
    {
        alpha_ = std::exp(-w_h_ * Ts_);
        y_prev_ = 0.0;
    }

    double step(double x_k) {
        double y_k = alpha_ * y_prev_ + (1.0 - alpha_) * x_k;
        y_prev_ = y_k;
        return y_k;
    }

private:
    double Ts_;
    double w_h_;
    double alpha_;
    double y_prev_;
};

class PIDWithRolloff {
public:
    PIDWithRolloff(double Kp, double Ki, double Kd,
                   double Ts, double w_h)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      Ts_(Ts), filt_(Ts, w_h),
      integ_(0.0), prev_meas_(0.0)
    {}

    double step(double ref, double meas_raw) {
        // filter measurement to suppress high-frequency noise
        double meas = filt_.step(meas_raw);

        double error = ref - meas;
        integ_ += error * Ts_;
        double deriv = (meas - prev_meas_) / Ts_; // derivative on measurement
        prev_meas_ = meas;

        // filtered derivative term reduces high-frequency gain
        double u = Kp_ * error + Ki_ * integ_ - Kd_ * deriv;
        return u;
    }

private:
    double Kp_, Ki_, Kd_;
    double Ts_;
    LowPassFilter filt_;
    double integ_;
    double prev_meas_;
};
