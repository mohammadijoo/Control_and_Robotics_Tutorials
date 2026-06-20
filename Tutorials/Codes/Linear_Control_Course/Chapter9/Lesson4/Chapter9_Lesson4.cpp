#include <iostream>
#include <cmath>

bool isStableGain(double K) {
    // Routh column for p(s,K) = s^3 + 6 s^2 + 8 s + K
    double r3 = 1.0;
    double r2 = 6.0;
    double r1 = (48.0 - K) / 6.0;
    double r0 = K;

    return (r3 > 0.0) && (r2 > 0.0) && (r1 > 0.0) && (r0 > 0.0);
}

int main() {
    for (double K = 0.0; K <= 60.0; K += 10.0) {
        std::cout << "K = " << K
                  << (isStableGain(K) ? " : stable\n" : " : unstable\n");
    }

    // Example policy for a robotic joint:
    double Kcrit = 48.0;
    double Kdesign = 0.4 * Kcrit; // design at 40% of critical gain
    std::cout << "Suggested design gain Kdesign = " << Kdesign
              << (isStableGain(Kdesign) ? " (stable)\n" : " (unstable)\n");
    return 0;
}
