#include <iostream>
#include <random>
#include <cmath>

bool trial_success(int numSamples, int dim, double radius) {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    // center at 0.5 in each coordinate
    std::vector<double> center(dim, 0.5);

    for (int i = 0; i < numSamples; ++i) {
        double dist2 = 0.0;
        for (int d = 0; d < dim; ++d) {
            double x = unif(gen);
            double diff = x - center[d];
            dist2 += diff * diff;
        }
        if (std::sqrt(dist2) <= radius) {
            return true; // at least one sample hits the goal tube
        }
    }
    return false;
}

double estimate_success_prob(int numSamples, int dim = 4,
                             double radius = 0.1, int trials = 1000) {
    int successCount = 0;
    for (int t = 0; t < trials; ++t) {
        if (trial_success(numSamples, dim, radius)) {
            ++successCount;
        }
    }
    return static_cast<double>(successCount) / trials;
}

int main() {
    for (int n : {10, 50, 100, 200, 500, 1000}) {
        double p = estimate_success_prob(n);
        std::cout << "n = " << n
                  << ", approx P(success) = " << p << std::endl;
    }
    return 0;
}
      
