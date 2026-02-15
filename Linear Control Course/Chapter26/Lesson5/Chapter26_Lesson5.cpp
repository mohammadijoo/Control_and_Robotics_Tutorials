#include <cmath>

class FirstOrderLowPass {
public:
    FirstOrderLowPass(double Ts, double tau_f)
    {
        // Backward Euler discretization of F(s) = 1 / (1 + tau_f s)
        double denom = Ts + tau_f;
        b0_ = Ts / denom;
        a1_ = -Ts / denom;
        y1_ = 0.0;
    }

    double filter(double u)
    {
        // y[k] = -a1*y[k-1] + b0*u[k]
        double y = -a1_ * y1_ + b0_ * u;
        y1_ = y;
        return y;
    }

private:
    double b0_;
    double a1_;
    double y1_;
};

// Example usage in a joint controller update function:
// FirstOrderLowPass vel_filter(Ts, 0.01);
// double vel_filtered = vel_filter.filter(vel_measured);
