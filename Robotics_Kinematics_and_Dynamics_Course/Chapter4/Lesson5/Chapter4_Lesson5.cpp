#include <array>
#include <cmath>

enum class Assumption {
    Ideal,
    Offset,
    Linearized
};

struct Planar2RModel {
    double l1{1.0};
    double l2{1.0};
    std::array<double, 2> q_offset{{0.0, 0.0}};

    std::array<double, 2> fk(std::array<double, 2> q,
                               Assumption assumption) const {
        if (assumption == Assumption::Offset) {
            q[0] += q_offset[0];
            q[1] += q_offset[1];
        }

        double q1 = q[0];
        double q2 = q[1];

        double x, y;
        if (assumption == Assumption::Linearized) {
            // Same small-angle linearization as in Python
            x = l1 + l2;
            y = l1 * q1 + l2 * (q1 + q2);
        } else {
            x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
            y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
        }
        return {x, y};
    }
};

// In practice, C++ robotics libraries such as KDL or Pinocchio
// use the same ideal rigid-link, ideal-joint modeling assumptions,
// with additional optional parameters for more detailed effects.
      
