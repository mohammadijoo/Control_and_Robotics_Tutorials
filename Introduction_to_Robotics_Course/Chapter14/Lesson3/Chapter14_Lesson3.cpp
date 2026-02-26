#include <vector>
#include <cmath>

// These functions could be called inside a ROS2 rclcpp node
// that subscribes to interaction logs and publishes usability metrics.

double shannonID(double D, double W) {
    return std::log2(1.0 + D / W);
}

double fittsMT(double D, double W, double a, double b) {
    return a + b * shannonID(D, W);
}

double throughput(const std::vector<double>& IDs,
                  const std::vector<double>& MTs) {
    double sumID = 0.0;
    double sumMT = 0.0;
    for (std::size_t i = 0; i < IDs.size(); ++i) {
        sumID += IDs[i];
        sumMT += MTs[i];
    }
    return sumID / sumMT;
}

double safeVelocity(double t_r, double a_max, double d_safe) {
    // Solve v^2/(2 a_max) + t_r v - d_safe = 0 for v >= 0.
    double A = 1.0 / (2.0 * a_max);
    double B = t_r;
    double C = -d_safe;
    double disc = B*B - 4.0*A*C;
    if (disc < 0.0) return 0.0;
    double sqrt_disc = std::sqrt(disc);
    double v1 = (-B + sqrt_disc) / (2.0 * A);
    double v2 = (-B - sqrt_disc) / (2.0 * A);
    double v = 1e9;
    if (v1 >= 0.0 && v1 < v) v = v1;
    if (v2 >= 0.0 && v2 < v) v = v2;
    if (v == 1e9) v = 0.0;
    return v;
}

int main() {
    std::vector<double> IDs{shannonID(0.2, 0.04),
                             shannonID(0.3, 0.05),
                             shannonID(0.4, 0.06)};
    std::vector<double> MTs{0.45, 0.50, 0.55};
    double TP = throughput(IDs, MTs);

    double t_r = 0.4;   // example reaction time [s]
    double a_max = 3.0; // m/s^2
    double d_safe = 1.5;
    double v_max = safeVelocity(t_r, a_max, d_safe);

    // In a real node, print or publish TP and v_max.
    return 0;
}
      
