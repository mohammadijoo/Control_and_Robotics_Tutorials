// Chapter19_Lesson4.cpp
// Compact AMR stress-test harness (C++17)

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

struct Episode {
    int success;
    double rmse;
    double margin;
    double score;
};

Episode simulate(int seed, double stress, const std::map<std::string, int>& mods) {
    std::mt19937 gen(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    auto norm = [&](double mu, double sigma) { return mu + sigma * n01(gen); };

    std::poisson_distribution<int> pd(2 + static_cast<int>(6 * stress));
    int dyn = pd(gen);
    int dropout = (u01(gen) < std::min(0.85, 0.05 + 0.75 * stress)) ? 1 : 0;

    double slip = std::fabs(norm(0.0, 0.02 + 0.12 * stress));
    double bias = std::fabs(norm(0.0, 0.01 + 0.08 * stress));
    double missing = 0.9 * (1 - mods.at("scan")) + 0.8 * (1 - mods.at("imu")) + 0.7 * (1 - mods.at("predict"));

    double rmse = 0.05 + 0.09 * stress + 0.04 * dyn + 1.4 * slip + 1.6 * bias + 0.10 * dropout + 0.12 * missing + norm(0.0, 0.02);
    if (rmse < 0.01) rmse = 0.01;

    double margin = 0.55 - 0.18 * stress - 0.01 * dyn - 0.07 * dropout - 0.05 * missing + 0.02 * mods.at("predict") + norm(0.0, 0.03);
    double pathRatio = 1.03 + 0.14 * stress + 0.02 * dyn + 0.08 * dropout + 0.05 * missing - 0.03 * mods.at("scan") + norm(0.0, 0.02);
    if (pathRatio < 1.0) pathRatio = 1.0;
    double ttc = 34 + 7 * pathRatio + 2.0 * dyn + 8.0 * stress + 3.5 * missing + norm(0.0, 1.5);
    if (ttc < 5.0) ttc = 5.0;

    double logit = 3.2 - 2.6 * stress - 2.2 * rmse + 1.0 * margin - 0.5 * (pathRatio - 1.0) - 0.7 * missing;
    double p = 1.0 / (1.0 + std::exp(-logit));
    int success = (u01(gen) < p) ? 1 : 0;

    double score = 100.0;
    score -= 35.0 * (1 - success);
    score -= 16.0 * std::max(0.0, rmse - 0.10);
    score -= 14.0 * std::max(0.0, 0.25 - margin);
    score -= 5.0 * std::max(0.0, pathRatio - 1.10);
    score -= 0.20 * std::max(0.0, ttc - 45.0);
    if (score < 0.0) score = 0.0;

    return {success, rmse, margin, score};
}

int main() {
    std::vector<double> stressGrid = {0.0, 0.4, 0.8};
    std::map<std::string, int> full = {{"scan", 1}, {"imu", 1}, {"predict", 1}};
    std::map<std::string, int> noScan = {{"scan", 0}, {"imu", 1}, {"predict", 1}};

    std::cout << std::fixed << std::setprecision(3);
    for (double s : stressGrid) {
        double succFull = 0.0, scoreFull = 0.0, rmseFull = 0.0;
        double deltaSum = 0.0;
        int n = 0;
        for (int seed = 100; seed < 160; ++seed) {
            Episode a = simulate(seed, s, full);
            Episode b = simulate(seed, s, noScan); // same seed -> paired
            succFull += a.success;
            scoreFull += a.score;
            rmseFull += a.rmse;
            deltaSum += (a.score - b.score);
            n++;
        }
        std::cout << "stress=" << s
                  << " fullSucc=" << succFull / n
                  << " fullScore=" << scoreFull / n
                  << " fullRMSE=" << rmseFull / n
                  << " pairedDelta(Full-NoScan)=" << deltaSum / n
                  << "\n";
    }
    return 0;
}
