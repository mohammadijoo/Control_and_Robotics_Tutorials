#include <stdexcept>

struct SpeedSeparationConfig {
    double reaction_time;  // T_r
    double max_decel;      // a_max > 0
    double safety_margin;  // d_safe
};

double stoppingDistance(double v, const SpeedSeparationConfig& cfg) {
    if (cfg.max_decel <= 0.0) {
        throw std::runtime_error("max_decel must be positive");
    }
    if (v < 0.0) v = 0.0;
    return v * cfg.reaction_time + 0.5 * v * v / cfg.max_decel;
}

double safetyFunction(double d, double v, const SpeedSeparationConfig& cfg) {
    double d_stop = stoppingDistance(v, cfg);
    return d - d_stop - cfg.safety_margin;
}

bool hazardActive(double d, double v, const SpeedSeparationConfig& cfg) {
    return safetyFunction(d, v, cfg) < 0.0;
}

// Placeholder for integration with low-level robot controller:
void emergencyStop() {
    // Send stop command over fieldbus (e.g., CAN, EtherCAT) or digital output.
}

void monitorStep(double d, double v, const SpeedSeparationConfig& cfg) {
    if (hazardActive(d, v, cfg)) {
        emergencyStop();
    }
}
      
