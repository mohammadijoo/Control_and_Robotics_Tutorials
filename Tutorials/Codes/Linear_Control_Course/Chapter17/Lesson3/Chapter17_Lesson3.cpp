#include <iostream>
#include <cmath>

// Compute delay margin from phase margin (deg) and gain crossover (rad/s)
double delayMargin(double phaseMarginDeg, double w_gc) {
    const double phaseMarginRad = phaseMarginDeg * M_PI / 180.0;
    return phaseMarginRad / w_gc;
}

int main() {
    double phaseMarginDeg = 45.0;   // e.g., from offline Bode plot
    double w_gc = 5.0;              // rad/s
    double T_d = delayMargin(phaseMarginDeg, w_gc);
    std::cout << "Delay margin (s): " << T_d << std::endl;

    // In a ROS node, this function can be used to check whether the estimated
    // communication + computation delay stays below T_d for a given controller.
    return 0;
}
