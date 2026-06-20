
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using Eigen::Vector2d;

struct PlanarArm {
    double l1{1.0};
    double l2{0.7};
    double q1_min{-M_PI/2.0}, q1_max{M_PI/2.0};
    double q2_min{-M_PI},      q2_max{M_PI};
    double x_obs{0.7}, y_obs{0.3}, r_obs{0.2};

    Vector2d forwardKinematics(const Vector2d& q) const {
        double q1 = q(0), q2 = q(1);
        double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
        double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
        return Vector2d(x, y);
    }

    bool inJointLimits(const Vector2d& q) const {
        double q1 = q(0), q2 = q(1);
        return (q1_min <= q1 && q1 <= q1_max) &&
               (q2_min <= q2 && q2 <= q2_max);
    }

    double obstacleConstraint(const Vector2d& q) const {
        Vector2d ee = forwardKinematics(q);
        double dx = ee(0) - x_obs;
        double dy = ee(1) - y_obs;
        return dx*dx + dy*dy - r_obs*r_obs; // should be >= 0
    }

    double groundConstraint(const Vector2d& q) const {
        Vector2d ee = forwardKinematics(q);
        return ee(1); // y >= 0
    }

    bool admissible(const Vector2d& q) const {
        if (!inJointLimits(q)) return false;
        if (obstacleConstraint(q) < 0.0) return false;
        if (groundConstraint(q) < 0.0) return false;
        return true;
    }
};

int main() {
    PlanarArm arm;
    Vector2d q(0.0, 0.0);
    std::cout << "Admissible? " << std::boolalpha
              << arm.admissible(q) << std::endl;
    return 0;
}
