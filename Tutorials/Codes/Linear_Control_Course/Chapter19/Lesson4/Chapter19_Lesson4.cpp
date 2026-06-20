#include <complex>
#include <cmath>
#include <iostream>

struct FirstOrderFactor {
    double z; // zero location (for numerator) or pole location (for denominator)
    bool isZero; // true if (s + z) factor in numerator

    std::complex<double> eval(double w) const {
        std::complex<double> jw(0.0, w);
        if (isZero) {
            return jw + z;
        } else {
            return 1.0 / (jw + z);
        }
    }
};

struct LagCompensator {
    double Kc;
    FirstOrderFactor zero;
    FirstOrderFactor pole;

    std::complex<double> eval(double w) const {
        return Kc * zero.eval(w) * pole.eval(w);
    }
};

struct LeadLag {
    double Kc;
    FirstOrderFactor leadZero;
    FirstOrderFactor leadPole;
    FirstOrderFactor lagZero;
    FirstOrderFactor lagPole;

    std::complex<double> eval(double w) const {
        std::complex<double> jw_resp = Kc *
            leadZero.eval(w) * leadPole.eval(w) *
            lagZero.eval(w) * lagPole.eval(w);
        return jw_resp;
    }
};

int main() {
    // Example parameters (from an offline Bode design)
    LagCompensator lag;
    lag.Kc = 1.0;
    lag.zero = {10.0, true};   // (s + 10) in numerator
    lag.pole = {1.0, false};   // 1 / (s + 1) in denominator

    double w = 5.0; // rad/s
    std::complex<double> G_lag = lag.eval(w);
    double mag = std::abs(G_lag);
    double phase = std::arg(G_lag); // radians

    std::cout << "Lag magnitude at w = " << w
              << " rad/s: " << mag << std::endl;
    std::cout << "Lag phase at w = " << w
              << " rad/s: " << phase << " rad" << std::endl;

    return 0;
}
