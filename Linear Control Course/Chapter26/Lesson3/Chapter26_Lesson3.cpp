#include <cmath>
#include <vector>
#include <iostream>

class FirstOrderLowPassFilter {
public:
    FirstOrderLowPassFilter(double tau, double dt)
    : tau_(tau), dt_(dt), y_(0.0)
    {
        alpha_ = std::exp(-dt_ / tau_);
    }

    double update(double y_meas) {
        y_ = alpha_ * y_ + (1.0 - alpha_) * y_meas;
        return y_;
    }

    void reset(double y_init) {
        y_ = y_init;
    }

private:
    double tau_;
    double dt_;
    double alpha_;
    double y_;
};

int main() {
    double dt   = 0.001;
    double tau  = 0.02;
    FirstOrderLowPassFilter filter(tau, dt);

    // Example: filter a simulated noisy signal
    std::vector<double> y_meas = {0.0, 0.1, 0.2, 0.15, 0.18, 0.19};
    for (double ym : y_meas) {
        double yf = filter.update(ym);
        std::cout << "y_meas = " << ym
                  << ", y_filt = " << yf << std::endl;
    }

    return 0;
}
