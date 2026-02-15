#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

struct JointLimits2R {
    std::array<double, 2> lower;
    std::array<double, 2> upper;
};

inline double wrapToPi(double angle) {
    const double twoPi = 2.0 * M_PI;
    angle = std::fmod(angle + M_PI, twoPi);
    if (angle < 0.0) {
        angle += twoPi;
    }
    return angle - M_PI;
}

bool ik2R(double x, double y,
          double l1, double l2,
          std::vector<std::array<double, 2> > &solutions,
          const JointLimits2R *limits = nullptr)
{
    solutions.clear();

    const double r2 = x * x + y * y;
    const double denom = 2.0 * l1 * l2;
    const double c2 = (r2 - l1 * l1 - l2 * l2) / denom;

    if (std::fabs(c2) > 1.0) {
        // No real solutions
        return false;
    }

    const double s2_pos = std::sqrt(std::max(0.0, 1.0 - c2 * c2));

    // Loop over elbow-up / elbow-down
    const double s2_candidates[2] = { s2_pos, -s2_pos };
    for (double s2 : s2_candidates) {
        const double theta2 = std::atan2(s2, c2);
        const double k1 = l1 + l2 * c2;
        const double k2 = l2 * s2;
        double theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        double t1 = wrapToPi(theta1);
        double t2 = wrapToPi(theta2);

        // Check joint limits if provided
        if (limits) {
            if (t1 < limits->lower[0] || t1 > limits->upper[0] ||
                t2 < limits->lower[1] || t2 > limits->upper[1]) {
                continue;
            }
        }

        solutions.push_back({t1, t2});
    }

    // Optional: remove duplicate solutions in singular cases
    if (solutions.size() == 2) {
        const double eps = 1e-9;
        const auto &q0 = solutions[0];
        const auto &q1 = solutions[1];
        if (std::fabs(q0[0] - q1[0]) < eps &&
            std::fabs(q0[1] - q1[1]) < eps) {
            solutions.pop_back();
        }
    }

    return !solutions.empty();
}
      
