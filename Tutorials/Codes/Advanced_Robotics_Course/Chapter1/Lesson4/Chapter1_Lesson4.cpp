#include <vector>
#include <cmath>
#include <cassert>

enum JointType { REVOLUTE = 0, PRISMATIC = 1 };

inline double wrapToPi(double angle)
{
    // Map angle to (-pi, pi]
    const double twoPi = 2.0 * M_PI;
    angle = std::fmod(angle + M_PI, twoPi);
    if (angle < 0.0)
        angle += twoPi;
    return angle - M_PI;
}

double configDistance(const std::vector<double>& q1,
                      const std::vector<double>& q2,
                      const std::vector<JointType>& jointTypes,
                      const std::vector<double>& weights)
{
    const std::size_t n = q1.size();
    assert(q2.size() == n);
    assert(jointTypes.size() == n);
    assert(weights.size() == n);

    double sumSq = 0.0;
    for (std::size_t i = 0; i < n; ++i)
    {
        double delta = q2[i] - q1[i];
        if (jointTypes[i] == REVOLUTE)
        {
            delta = wrapToPi(delta);
        }
        const double wdelta = weights[i] * delta;
        sumSq += wdelta * wdelta;
    }
    return std::sqrt(sumSq);
}

// Example usage in a planning library:
// double distance(const State* s1, const State* s2) {
//     // Extract q1, q2 from state representation and call configDistance(...)
// }
      
