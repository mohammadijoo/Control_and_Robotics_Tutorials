#include <iostream>
#include <vector>
#include <cmath>

struct FrequencyResponse {
    double Mr;
    double wr;
    double wb;
};

FrequencyResponse analyzeSecondOrder(double wn, double zeta) {
    // Log-spaced frequency grid
    const int N = 2000;
    double w_min = 0.1;
    double w_max = 100.0;
    std::vector<double> w(N);
    for (int k = 0; k < N; ++k) {
        double alpha = static_cast<double>(k) / (N - 1);
        w[k] = w_min * std::pow(w_max / w_min, alpha);
    }

    // Compute magnitude
    std::vector<double> mag(N);
    for (int k = 0; k < N; ++k) {
        double wi = w[k];
        double x = wi / wn;
        double denom = std::pow(1.0 - x * x, 2.0) + std::pow(2.0 * zeta * x, 2.0);
        mag[k] = 1.0 / std::sqrt(denom);
    }

    // Resonant peak
    double Mr = mag[0];
    double wr = w[0];
    for (int k = 1; k < N; ++k) {
        if (mag[k] > Mr) {
            Mr = mag[k];
            wr = w[k];
        }
    }

    // -3 dB bandwidth
    double target = mag[0] / std::sqrt(2.0);
    double wb = w.back();
    for (int k = 0; k < N; ++k) {
        if (mag[k] <= target) {
            wb = w[k];
            break;
        }
    }

    return {Mr, wr, wb};
}

int main() {
    double wn = 10.0;
    double zeta = 0.3;
    FrequencyResponse fr = analyzeSecondOrder(wn, zeta);

    std::cout << "Mr = " << fr.Mr
              << ", wr = " << fr.wr
              << " rad/s, wb = " << fr.wb
              << " rad/s" << std::endl;

    // In a ROS-based robot controller, such analysis can be used offline
    // to check that the joint-space PD gains produce acceptable resonance and bandwidth.

    return 0;
}
