#include <iostream>

// Compute proportional gain for a first-order plant G(s) = 1/(tau s + 1)
// to achieve target closed-loop bandwidth omega_b (rad/s).
double computeGainFromBandwidth(double tau, double omega_b) {
    // K = tau * omega_b - 1
    return tau * omega_b - 1.0;
}

// Compute corresponding closed-loop bandwidth (first-order model)
double closedLoopBandwidth(double tau, double K) {
    return (1.0 + K) / tau;
}

int main() {
    double tau = 0.05;          // identified time constant (s)
    double omega_b_target = 40.0; // target bandwidth (rad/s)

    double K = computeGainFromBandwidth(tau, omega_b_target);
    std::cout << "Designed gain K = " << K << std::endl;

    double omega_b = closedLoopBandwidth(tau, K);
    std::cout << "Closed-loop bandwidth ≈ " << omega_b << " rad/s" << std::endl;

    // In a robotics controller, K would be used as the proportional gain in a joint servo.
    return 0;
}
