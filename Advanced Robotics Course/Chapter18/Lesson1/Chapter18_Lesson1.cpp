#include <vector>
#include <cmath>
#include <cstddef>

double wrapAngle(double angle)
{
    const double pi = 3.141592653589793;
    double a = std::fmod(angle + pi, 2.0 * pi);
    if (a < 0.0) {
        a += 2.0 * pi;
    }
    return a - pi;
}

// weights.size() == q1.size() == q2.size()
// revolute[i] is true if joint i is revolute.
double configDistance(const std::vector<double> &q1,
                      const std::vector<double> &q2,
                      const std::vector<double> &weights,
                      const std::vector<bool> &revolute)
{
    const std::size_t n = q1.size();
    double sum = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        double diff = q2[i] - q1[i];
        if (revolute[i]) {
            diff = wrapAngle(diff);
        }
        sum += weights[i] * diff * diff;
    }
    return std::sqrt(sum);
}
      
