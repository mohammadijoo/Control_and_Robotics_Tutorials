#include <cmath>
#include <vector>
#include <stdexcept>

struct TOPPResult1D {
    std::vector<double> s;
    std::vector<double> sdot;
    std::vector<double> t;
};

TOPPResult1D topp1D(double s_f, int N, double v_max, double a_max) {
    if (N <= 0) {
        throw std::invalid_argument("N must be positive");
    }
    TOPPResult1D res;
    res.s.resize(N + 1);
    res.sdot.assign(N + 1, 0.0);
    res.t.assign(N + 1, 0.0);

    // Discretize s
    double ds = s_f / static_cast<double>(N);
    for (int k = 0; k <= N; ++k) {
        res.s[k] = ds * static_cast<double>(k);
    }

    // Velocity limit curve (constant here)
    std::vector<double> sdot_max(N + 1, v_max);

    // Forward pass
    for (int k = 0; k < N; ++k) {
        double sdot_sq = res.sdot[k] * res.sdot[k] + 2.0 * a_max * ds;
        double sdot_cand = std::sqrt(std::max(0.0, sdot_sq));
        res.sdot[k + 1] = std::min(sdot_cand, sdot_max[k + 1]);
    }

    // Backward pass
    for (int k = N - 1; k >= 0; --k) {
        double sdot_sq = res.sdot[k + 1] * res.sdot[k + 1] + 2.0 * a_max * ds;
        double sdot_cand = std::sqrt(std::max(0.0, sdot_sq));
        res.sdot[k] = std::min({res.sdot[k], sdot_cand, sdot_max[k]});
    }

    // Time stamps (trapezoidal rule)
    for (int k = 0; k < N; ++k) {
        double denom = res.sdot[k] + res.sdot[k + 1];
        if (denom <= 1e-9) {
            throw std::runtime_error("Infeasible profile: zero velocity segment");
        }
        res.t[k + 1] = res.t[k] + 2.0 * ds / denom;
    }

    return res;
}

// Example usage:
// int main() {
//     auto res = topp1D(1.0, 100, 1.0, 2.0);
//     std::cout << "Final time T = " << res.t.back() << std::endl;
// }
      
