#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

struct Sensor {
    double rmin, rmax, fs, Delta, sigma_v, cost;
};

int main() {
    std::vector<Sensor> S = {
        {0.0, 5.0, 200.0, 0.01, 0.02, 50.0},
        {0.1, 10.0, 60.0, 0.05, 0.05, 20.0},
        {0.0, 8.0, 120.0, 0.02, 0.03, 35.0}
    };

    double x_min_req = 0.0, x_max_req = 6.0;
    double B = 40.0, delta_x = 0.03, eps_max = 0.05;

    std::vector<int> feasible_idx;
    std::vector<double> sigma_tot, latency, cost;

    for (int i=0; i<(int)S.size(); ++i) {
        auto s = S[i];
        bool ok = (s.rmin <= x_min_req && s.rmax >= x_max_req);
        ok = ok && (s.fs >= 2*B);
        ok = ok && (s.Delta <= delta_x);
        double stot = std::sqrt(s.sigma_v*s.sigma_v + s.Delta*s.Delta/12.0);
        ok = ok && (stot <= eps_max);

        if (ok) {
            feasible_idx.push_back(i);
            sigma_tot.push_back(stot);
            latency.push_back(1.0/s.fs);
            cost.push_back(s.cost);
        }
    }

    if (feasible_idx.empty()) {
        std::cout << "No feasible sensors\n";
        return 0;
    }

    auto normalize = [](const std::vector<double>& v){
        double mn = *std::min_element(v.begin(), v.end());
        double mx = *std::max_element(v.begin(), v.end());
        Eigen::VectorXd out(v.size());
        for (int i=0;i<(int)v.size();++i)
            out(i) = (v[i]-mn)/(mx-mn + 1e-12);
        return out;
    };

    Eigen::VectorXd a1 = normalize(sigma_tot);
    Eigen::VectorXd a2 = normalize(latency);
    Eigen::VectorXd a3 = normalize(cost);

    Eigen::MatrixXd A(a1.size(),3);
    A.col(0)=a1; A.col(1)=a2; A.col(2)=a3;

    Eigen::Vector3d w(0.5, 0.3, 0.2);
    Eigen::VectorXd scores = A*w;

    Eigen::Index best_local;
    scores.minCoeff(&best_local);
    int best_global = feasible_idx[(int)best_local];

    std::cout << "Best sensor index: " << best_global << "\n";
    return 0;
}
