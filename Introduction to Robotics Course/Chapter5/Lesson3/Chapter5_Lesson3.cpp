
#include <iostream>
#include <vector>
#include <random>
#include <cmath>

int main() {
    double l1 = 1.0, l2 = 0.7;
    double q1_min = -M_PI, q1_max = M_PI;
    double q2_min = -M_PI, q2_max = M_PI;
    int N = 10000;

    std::mt19937 gen(0);
    std::uniform_real_distribution<double> d1(q1_min, q1_max);
    std::uniform_real_distribution<double> d2(q2_min, q2_max);

    std::vector<double> xs, ys;
    xs.reserve(N); ys.reserve(N);

    for (int k = 0; k < N; ++k) {
        double q1 = d1(gen);
        double q2 = d2(gen);
        double x = l1*std::cos(q1) + l2*std::cos(q1 + q2);
        double y = l1*std::sin(q1) + l2*std::sin(q1 + q2);
        xs.push_back(x);
        ys.push_back(y);
    }

    // Print a few sample points (plot externally if desired)
    for (int k = 0; k < 10; ++k) {
        std::cout << xs[k] << " " << ys[k] << std::endl;
    }
    return 0;
}
      