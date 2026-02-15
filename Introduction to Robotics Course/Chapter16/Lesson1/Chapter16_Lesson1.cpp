#include <vector>
#include <cmath>

struct Vec3 {
    double x, y, z;
};

double distance(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

class SafetyMonitor {
public:
    SafetyMonitor(double d_min, int stop_steps)
        : d_min_(d_min),
          stop_steps_(stop_steps),
          steps_since_obstacle_(-1) {}

    void update(const Vec3& p_r, const Vec3& p_h, double v, bool obstacle_detected) {
        // Invariant: minimum separation
        double d = distance(p_r, p_h);
        if (d < d_min_) {
            // Requirement violation: trigger emergency stop
            triggerEmergencyStop("Separation distance violated");
        }

        // Response-type requirement: if obstacle detected, stop within stop_steps_
        if (obstacle_detected) {
            if (steps_since_obstacle_ < 0) {
                steps_since_obstacle_ = 0;
            }
        }

        if (steps_since_obstacle_ >= 0) {
            if (v > 0.0 && steps_since_obstacle_ > stop_steps_) {
                triggerEmergencyStop("Reaction time requirement violated");
            }
            ++steps_since_obstacle_;
        }
    }

private:
    void triggerEmergencyStop(const char* reason) {
        // Implementation-specific: log, cut power, activate brakes, etc.
        // For illustration, we might set flags or send messages.
        // In a real robot, this function must be thoroughly tested.
    }

    double d_min_;
    int stop_steps_;
    int steps_since_obstacle_;
};
      
