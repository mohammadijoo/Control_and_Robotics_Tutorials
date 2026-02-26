#include <vector>
#include <cmath>
#include <iostream>

double compute_ISE(const std::vector<double>& t,
                   const std::vector<double>& r,
                   const std::vector<double>& y) {
    std::size_t N = t.size();
    if (N == 0) return 0.0;

    double J = 0.0;
    for (std::size_t k = 0; k + 1 < N; ++k) {
        double dt = t[k + 1] - t[k];
        double e = r[k] - y[k];
        J += e * e * dt;
    }
    return J;
}

double compute_RMS(const std::vector<double>& r,
                   const std::vector<double>& y) {
    std::size_t N = r.size();
    if (N == 0) return 0.0;

    double sum_sq = 0.0;
    for (std::size_t k = 0; k < N; ++k) {
        double e = r[k] - y[k];
        sum_sq += e * e;
    }
    return std::sqrt(sum_sq / static_cast<double>(N));
}

int main() {
    // In practice, load t, r, y from a CSV or from a ROS bag file.
    std::vector<double> t   = {0.0, 0.01, 0.02};
    std::vector<double> r   = {1.0, 1.0, 1.0};
    std::vector<double> y   = {0.8, 0.9, 0.95};

    double J_ISE = compute_ISE(t, r, y);
    double e_RMS = compute_RMS(r, y);

    std::cout << "ISE = " << J_ISE << "\n";
    std::cout << "RMS error = " << e_RMS << "\n";

    return 0;
}
      
