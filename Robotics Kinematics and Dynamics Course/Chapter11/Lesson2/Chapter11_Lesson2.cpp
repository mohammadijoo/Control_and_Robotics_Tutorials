#include <array>
#include <cmath>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

// Compute Coriolis matrix C(q, q_dot) for 2R elbow manipulator
// with inertia matrix
// M11 = a1 + 2 a2 cos(q2)
// M12 = a3 + a2 cos(q2)
// M22 = a3
Mat2 coriolis2R(const Vec2& q, const Vec2& dq,
                double a1, double a2, double a3)
{
    (void)a1; // unused in Coriolis
    (void)a3; // unused in Coriolis

    double q2  = q[1];
    double dq1 = dq[0];
    double dq2 = dq[1];

    double s2  = std::sin(q2);
    double h   = -a2 * s2;  // scalar h(q) often used in robotics

    Mat2 C{};
    // One consistent convention:
    // C11 = h * dq2
    // C12 = h * (dq1 + dq2)
    // C21 = -h * dq1
    // C22 = 0.0
    C[0][0] = h * dq2;
    C[0][1] = h * (dq1 + dq2);
    C[1][0] = -h * dq1;
    C[1][1] = 0.0;

    return C;
}

// Example usage in a torque computation:
// tau = M(q) q_ddot + C(q, q_dot) q_dot + g(q)
Vec2 coriolisTorque2R(const Vec2& q, const Vec2& dq,
                      double a1, double a2, double a3)
{
    Mat2 C = coriolis2R(q, dq, a1, a2, a3);
    Vec2 result{0.0, 0.0};
    result[0] = C[0][0] * dq[0] + C[0][1] * dq[1];
    result[1] = C[1][0] * dq[0] + C[1][1] * dq[1];
    return result;
}
      
