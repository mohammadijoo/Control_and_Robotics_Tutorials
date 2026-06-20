#include <iostream>
#include <cmath>

class SafetyEnvelope1D {
public:
    SafetyEnvelope1D(double reaction_time,
                     double max_decel,
                     double no_go_distance,
                     double margin = 0.05)
        : Tr_(reaction_time),
          amax_(max_decel),
          dng_(no_go_distance),
          margin_(margin) {}

    double stopping_distance(double v) const {
        // d_stop = v*Tr + v^2/(2*a_max)
        return v * Tr_ + (v * v) / (2.0 * amax_);
    }

    bool is_safe(double distance, double v) const {
        if (v < 0.0) {
            return true; // moving away
        }
        double d_available = distance - dng_ - margin_;
        if (d_available < 0.0) d_available = 0.0;
        double d_required = stopping_distance(v);
        return d_required <= d_available;
    }

private:
    double Tr_, amax_, dng_, margin_;
};

int main() {
    SafetyEnvelope1D env(0.2, 3.0, 0.4);
    double distance = 1.0;
    double v = 0.8;

    if (env.is_safe(distance, v)) {
        std::cout << "Command accepted (C++)." << std::endl;
    } else {
        std::cout << "Safety stop (C++)." << std::endl;
    }
    return 0;
}
      
