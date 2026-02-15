#include <iostream>
#include <cmath>

// Simple discrete-time PI + lead controller for one servo axis.
class ServoController {
public:
    ServoController(double Kc, double zi, double zl, double pl, double Ts)
        : Kc_(Kc), zi_(zi), zl_(zl), pl_(pl), Ts_(Ts),
          x_int_(0.0), x_lead_(0.0) {}

    // One control step: r = reference, y = measured position
    double step(double r, double y) {
        double e = r - y;

        // Integral state update (backward Euler approximation)
        x_int_ += Ts_ * zi_ * e;

        // Lead filter (first-order, bilinear-style discretization)
        double a = 1.0 + pl_ * Ts_;
        double b = zl_ * Ts_;
        double u_lead = (b * e + x_lead_) / a;
        x_lead_ = b * e + (1.0 - pl_ * Ts_) * x_lead_;

        // Control output
        double u = Kc_ * (e + x_int_ + u_lead);
        return u;
    }

private:
    double Kc_, zi_, zl_, pl_, Ts_;
    double x_int_;
    double x_lead_;
};

int main() {
    double Ts = 0.001;  // 1 kHz servo loop
    ServoController ctrl(12.0, 0.8, 6.0, 1.5, Ts);

    double r = 1.0;     // unit position step
    double y = 0.0;     // measured position (here a dummy)
    for (int k = 0; k < 5000; ++k) {
        double u = ctrl.step(r, y);
        // In a real robot, send u to motor drive and read updated y.
        // Here we just print u occasionally:
        if (k % 1000 == 0) {
            std::cout << "k = " << k << ", u = " << u << std::endl;
        }
    }
    return 0;
}
