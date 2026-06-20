#include <iostream>
#include <cmath>

struct Specs {
    double zeta_min;
    double Ts_max;
};

bool specs_satisfied(double a, double K, const Specs& specs) {
    double disc = a * a - 4.0 * K;
    // Require underdamped behavior: disc < 0
    if (disc >= 0.0) return false;

    double sigma = -0.5 * a;
    double omega = 0.5 * std::sqrt(-disc);
    double wn = std::sqrt(sigma * sigma + omega * omega);

    if (wn == 0.0) return false;

    double zeta = -sigma / wn;
    double Ts = 4.0 / (-sigma);

    return (zeta >= specs.zeta_min) && (Ts <= specs.Ts_max);
}

int main() {
    double a = 4.0;           // from plant G(s) = 1 / (s (s + a))
    Specs specs{0.6, 1.0};    // zeta_min, Ts_max

    double K_min = 0.1, K_max = 20.0;
    int N = 200;
    for (int i = 0; i <= N; ++i) {
        double K = K_min + (K_max - K_min) * i / static_cast<double>(N);
        if (specs_satisfied(a, K, specs)) {
            std::cout << "Acceptable gain K = " << K << std::endl;
            break;
        }
    }
    return 0;
}
