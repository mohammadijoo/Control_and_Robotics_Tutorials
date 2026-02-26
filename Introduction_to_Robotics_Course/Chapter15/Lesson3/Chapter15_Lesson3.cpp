#include <cmath>
#include <iostream>
#include <stdexcept>

double reliabilityExponential(double lambda_d, double t) {
    if (lambda_d < 0.0 || t < 0.0) {
        throw std::invalid_argument("lambda_d and t must be non-negative");
    }
    return std::exp(-lambda_d * t);
}

double kOutOfN(double Rc, int n, int k) {
    if (Rc < 0.0 || Rc > 1.0) {
        throw std::invalid_argument("Rc must be in [0,1]");
    }
    if (k < 1 || k > n) {
        throw std::invalid_argument("Need 1 <= k <= n");
    }

    auto comb = [](int n_, int r_) -> double {
        if (r_ < 0 || r_ > n_) return 0.0;
        if (r_ == 0 || r_ == n_) return 1.0;
        double c = 1.0;
        for (int i = 1; i <= r_; ++i) {
            c *= static_cast<double>(n_ - r_ + i) / static_cast<double>(i);
        }
        return c;
    };

    double R_sys = 0.0;
    for (int i = k; i <= n; ++i) {
        double c = comb(n, i);
        R_sys += c * std::pow(Rc, i) * std::pow(1.0 - Rc, n - i);
        ;
    }
    return R_sys;
}

int main() {
    double lambda_d = 1e-6;
    double T_I = 8760.0;

    double Rc = reliabilityExponential(lambda_d, T_I);
    double R1 = kOutOfN(Rc, 1, 1);
    double R2 = kOutOfN(Rc, 2, 1);

    std::cout << "Rc(T_I)  = " << Rc << "\n";
    std::cout << "R_1oo1   = " << R1 << "\n";
    std::cout << "R_1oo2   = " << R2 << "\n";
    return 0;
}
      
