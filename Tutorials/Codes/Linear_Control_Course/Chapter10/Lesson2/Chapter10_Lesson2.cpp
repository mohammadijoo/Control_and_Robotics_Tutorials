#include <iostream>
#include <cmath>

// Overshoot for G(s) = 1 / (s (s + 2)) with P gain K > 1
// Mp(K) = 100 * exp(-pi / sqrt(K - 1))
double overshootFromK(double K) {
    if (K <= 1.0) {
        return 0.0; // overdamped, no oscillatory overshoot
    }
    return 100.0 * std::exp(-M_PI / std::sqrt(K - 1.0));
}

// Solve by simple line search for a K satisfying Mp(K) <= Mp_max
double selectGain(double Mp_max, double K_min, double K_max, double dK) {
    double bestK = -1.0;
    for (double K = K_min; K <= K_max; K += dK) {
        double Mp = overshootFromK(K);
        if (Mp <= Mp_max) {
            bestK = K;
            break;
        }
    }
    return bestK;
}

int main() {
    double Mp_max = 10.0;       // 10 percent overshoot
    double K_min  = 1.01;       // avoid K <= 1 (no complex poles)
    double K_max  = 20.0;
    double dK     = 0.001;

    double K_star = selectGain(Mp_max, K_min, K_max, dK);

    if (K_star > 0.0) {
        std::cout << "Selected gain K* ≈ " << K_star << std::endl;

        // Closed-loop poles are s = -1 ± j sqrt(K - 1)
        double sigma = -1.0;
        double wd    = std::sqrt(K_star - 1.0);
        double zeta  = 1.0 / std::sqrt(K_star);
        double Ts    = 4.0; // 2% settling time ≈ 4 s for this plant

        std::cout << "Dominant poles: s = " << sigma
                  << " ± j " << wd << std::endl;
        std::cout << "Damping ratio zeta ≈ " << zeta << std::endl;
        std::cout << "Approximate Ts(2%) ≈ " << Ts << " s" << std::endl;
    } else {
        std::cout << "No gain in the search range satisfies the overshoot bound."
                  << std::endl;
    }

    return 0;
}
