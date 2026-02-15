#include <iostream>
#include <vector>
#include <cmath>

double seriesReliability(const std::vector<double> &lambdas, double t) {
    double lam_sum = 0.0;
    for (double lam : lambdas) {
        lam_sum += lam;
    }
    return std::exp(-lam_sum * t);
}

int main() {
    std::vector<double> lambdas{1e-4, 2e-4, 1.5e-4};
    double t = 1000.0;
    double R = seriesReliability(lambdas, t);
    std::cout << "Series reliability at t=" << t
              << " h is " << R << std::endl;
    return 0;
}
      
