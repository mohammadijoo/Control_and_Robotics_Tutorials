#include <iostream>
#include <cmath>
#include <utility>
#include <vector>

struct Solution2R {
    double q1;
    double q2;
};

std::vector<Solution2R> ik2R(double l1, double l2,
                             double x_d, double y_d,
                             double tol = 1e-9)
{
    std::vector<Solution2R> sols;

    double d_sq = x_d * x_d + y_d * y_d;
    double d = std::sqrt(d_sq);

    if (d > l1 + l2 + tol || d < std::fabs(l1 - l2) - tol) {
        // unreachable
        return sols;
    }

    double c2 = (d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;

    double disc = std::max(0.0, 1.0 - c2 * c2);
    double s2_candidates[2] = { std::sqrt(disc), -std::sqrt(disc) };

    for (double s2 : s2_candidates) {
        double q2 = std::atan2(s2, c2);

        double k1 = l1 + l2 * c2;
        double k2 = l2 * s2;

        double phi = std::atan2(y_d, x_d);
        double psi = std::atan2(k2, k1);
        double q1 = phi - psi;

        sols.push_back({q1, q2});
    }
    return sols;
}

int main()
{
    double l1 = 1.0, l2 = 0.6;
    double x_d = 1.0, y_d = 0.4;

    auto sols = ik2R(l1, l2, x_d, y_d);
    if (sols.empty()) {
        std::cout << "Target unreachable\n";
    } else {
        for (std::size_t i = 0; i < sols.size(); ++i) {
            std::cout << "Solution " << i
                      << ": q1 = " << sols[i].q1
                      << ", q2 = " << sols[i].q2 << "\n";
        }
    }
    return 0;
}
      
