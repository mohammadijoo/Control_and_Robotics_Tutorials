#include <iostream>
#include <vector>

class LowPassFilter {
public:
    LowPassFilter(double dt, double tau)
        : alpha(dt/(tau + dt)), y_prev(0.0) {}

    double step(double x) {
        double y = alpha * x + (1.0 - alpha) * y_prev;
        y_prev = y;
        return y;
    }
private:
    double alpha;
    double y_prev;
};

int main() {
    double dt = 0.005;   // 200 Hz
    double tau = 0.08;   // time constant
    LowPassFilter lpf(dt, tau);

    std::vector<double> x_m = {0.0, 0.1, 0.3, 0.2, 0.5, 0.4}; // example samples
    for (double x : x_m) {
        std::cout << lpf.step(x) << std::endl;
    }
    return 0;
}
      