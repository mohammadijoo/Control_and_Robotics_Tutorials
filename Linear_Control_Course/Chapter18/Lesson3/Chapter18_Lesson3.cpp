#include <cmath>

class LowPassFilter {
public:
    LowPassFilter(double tau_f, double Ts)
    {
        setParameters(tau_f, Ts);
        y_prev_ = 0.0;
    }

    void setParameters(double tau_f, double Ts)
    {
        tau_f_ = tau_f;
        Ts_ = Ts;
        alpha_ = std::exp(-Ts_ / tau_f_);
    }

    double filter(double y_meas)
    {
        // y_f[k] = alpha * y_f[k-1] + (1 - alpha) * y[k]
        double y_f = alpha_ * y_prev_ + (1.0 - alpha_) * y_meas;
        y_prev_ = y_f;
        return y_f;
    }

private:
    double tau_f_;
    double Ts_;
    double alpha_;
    double y_prev_;
};

// Example usage in a robot joint control loop:
// double y_meas = readEncoder();
// double y_filtered = lpf.filter(y_meas);
// double error = q_ref - y_filtered;
// double u = Kp * error + Kd * (error - error_prev) / Ts;
