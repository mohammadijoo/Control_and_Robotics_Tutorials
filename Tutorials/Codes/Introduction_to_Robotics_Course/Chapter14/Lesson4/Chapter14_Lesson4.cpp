#include <iostream>
#include <vector>

std::vector<double> update_trust(const std::vector<int> &successes,
                                   double lambda, double T0) {
    double T = T0;
    std::vector<double> history;
    history.push_back(T);
    for (int s : successes) {
        T = (1.0 - lambda) * T + lambda * static_cast<double>(s);
        history.push_back(T);
    }
    return history;
}

int main() {
    std::vector<int> seq{1, 1, 0, 1, 1, 1, 0, 1, 1, 1};
    double lambda = 0.3;
    double T0 = 0.2;

    auto trust_traj = update_trust(seq, lambda, T0);
    for (std::size_t k = 0; k < trust_traj.size(); ++k) {
        std::cout << "Step " << k
                  << ": T = " << trust_traj[k] << '\n';
    }
    return 0;
}
      
