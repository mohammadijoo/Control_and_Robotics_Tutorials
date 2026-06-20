#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Wrench = Eigen::Vector3d;

Eigen::Vector3d randomUnitVector3(std::mt19937 &gen) {
    std::normal_distribution<double> dist(0.0, 1.0);
    Eigen::Vector3d v(dist(gen), dist(gen), dist(gen));
    double n = v.norm();
    if (n < 1e-12) {
        return Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    return v / n;
}

double ferrariCannyApprox(const std::vector<Wrench> &wrenches,
                          int numDirs,
                          std::mt19937 &gen) {
    double q_val = std::numeric_limits<double>::infinity();
    for (int k = 0; k < numDirs; ++k) {
        Eigen::Vector3d d = randomUnitVector3(gen);
        double q_dir = -std::numeric_limits<double>::infinity();
        for (const auto &w : wrenches) {
            double s = d.dot(w);
            if (s > q_dir) {
                q_dir = s;
            }
        }
        if (q_dir < q_val) {
            q_val = q_dir;
        }
    }
    return q_val;
}

std::pair<double, double> robustQualityMC(
    const std::vector<Wrench> &W0,
    double sigma_w,
    int numSamples,
    double delta) {

    std::mt19937 gen(42);
    std::normal_distribution<double> noiseDist(0.0, sigma_w);

    std::vector<double> qVals;
    qVals.reserve(numSamples);

    for (int s = 0; s < numSamples; ++s) {
        std::vector<Wrench> W;
        W.reserve(W0.size());
        for (const auto &w0 : W0) {
            Wrench w = w0;
            w[0] += noiseDist(gen);
            w[1] += noiseDist(gen);
            w[2] += noiseDist(gen);
            W.push_back(w);
        }
        double q = ferrariCannyApprox(W, 64, gen);
        qVals.push_back(q);
    }

    // compute mean
    double mean = 0.0;
    for (double q : qVals) {
        mean += q;
    }
    mean /= static_cast<double>(qVals.size());

    // compute empirical delta-quantile
    std::sort(qVals.begin(), qVals.end());
    int idx = static_cast<int>(delta * qVals.size());
    idx = std::max(0, std::min(idx, static_cast<int>(qVals.size()) - 1));
    double q_delta = qVals[idx];

    return {mean, q_delta};
}

int main() {
    // Example nominal wrenches
    std::vector<Wrench> W0;
    W0.emplace_back(1.0,  0.8,  0.2);
    W0.emplace_back(-1.0, 0.8, -0.2);
    W0.emplace_back(0.8, -0.8,  0.3);
    W0.emplace_back(-0.8,-0.8, -0.3);

    auto result = robustQualityMC(W0, 0.05, 500, 0.1);
    std::cout << "Expected quality: " << result.first << std::endl;
    std::cout << "0.1-quantile quality: " << result.second << std::endl;
    return 0;
}
      
