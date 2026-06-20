#include <cmath>
#include <vector>
#include <utility>

struct AnglePair {
    double theta1;
    double theta2;
};

std::vector<AnglePair> ik2R(double x, double y, double l1, double l2, double tol = 1e-9) {
    std::vector<AnglePair> sols;
    double r2 = x*x + y*y;
    if (r2 > (l1 + l2)*(l1 + l2) + tol || r2 < (l1 - l2)*(l1 - l2) - tol) {
        return sols; // empty
    }

    double c2 = (r2 - l1*l1 - l2*l2) / (2.0 * l1 * l2);
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;

    double s2_abs = std::sqrt(std::max(0.0, 1.0 - c2*c2));
    double phi = std::atan2(y, x);

    for (double s2 : {s2_abs, -s2_abs}) {
        double theta2 = std::atan2(s2, c2);
        double k1 = l1 + l2 * c2;
        double k2 = l2 * s2;
        double psi = std::atan2(k2, k1);
        double theta1 = phi - psi;
        sols.push_back({theta1, theta2});
    }
    return sols;
}
      
