// Chapter18_Lesson4.cpp
// Weather/Lighting Effects on Perception (Outdoor and Field AMR)
// A compact confidence-aware fusion example (scalar position) in C++.

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <random>

struct ScalarKalman {
    double x{0.0};
    double P{1.0};
    double Q{0.01};

    void predict() { P += Q; }

    double update(double z, double R) {
        const double S = P + R;
        const double K = P / S;
        const double innov = z - x;
        x = x + K * innov;
        P = (1.0 - K) * P;
        return (innov * innov) / S;
    }
};

double cameraConfidence(double fog, double lux, double rain) {
    double q = std::pow(lux, 0.7) * std::exp(-2.2 * fog) * std::exp(-1.4 * rain);
    return std::max(0.05, std::min(1.0, q));
}

double lidarConfidence(double fog, double rain) {
    double q = std::exp(-1.3 * fog) * std::exp(-0.6 * rain);
    return std::max(0.10, std::min(1.0, q));
}

int main() {
    std::mt19937 gen(42);
    std::normal_distribution<double> wproc(0.0, 0.06);

    const int T = 220;
    const double Rcam0 = 0.60 * 0.60;
    const double Rlidar0 = 0.25 * 0.25;

    std::vector<double> xTrue(T, 0.0);
    for (int k = 1; k < T; ++k) {
        xTrue[k] = xTrue[k - 1] + 0.10 + wproc(gen);
    }

    ScalarKalman kfAdaptive;
    ScalarKalman kfFixed;
    kfAdaptive.Q = 0.02;
    kfFixed.Q = 0.02;

    double sqErrAdaptive = 0.0;
    double sqErrFixed = 0.0;
    double nisCamMean = 0.0;
    double nisLidarMean = 0.0;

    for (int k = 0; k < T; ++k) {
        const double fog = 0.08 + 0.35 * std::exp(-0.5 * std::pow((k - 100) / 20.0, 2.0));
        const double rain = 0.03 + 0.20 * std::exp(-0.5 * std::pow((k - 160) / 16.0, 2.0));
        const double lux = (k < 130) ? 1.0 : 0.28;

        const double qc = cameraConfidence(fog, lux, rain);
        const double ql = lidarConfidence(fog, rain);

        const double camStd = std::sqrt(Rcam0 / qc);
        const double lidarStd = std::sqrt(Rlidar0 / ql);

        std::normal_distribution<double> nCam(0.0, camStd);
        std::normal_distribution<double> nLidar(0.0, lidarStd);

        const double zCam = xTrue[k] + nCam(gen);
        const double zLidar = xTrue[k] + nLidar(gen);

        kfAdaptive.predict();
        kfFixed.predict();

        const double nisCam = kfAdaptive.update(zCam, Rcam0 / qc);
        const double nisLidar = kfAdaptive.update(zLidar, Rlidar0 / ql);

        kfFixed.update(zCam, Rcam0);
        kfFixed.update(zLidar, Rlidar0);

        nisCamMean += nisCam;
        nisLidarMean += nisLidar;
        sqErrAdaptive += std::pow(kfAdaptive.x - xTrue[k], 2.0);
        sqErrFixed += std::pow(kfFixed.x - xTrue[k], 2.0);
    }

    nisCamMean /= static_cast<double>(T);
    nisLidarMean /= static_cast<double>(T);

    const double rmseAdaptive = std::sqrt(sqErrAdaptive / static_cast<double>(T));
    const double rmseFixed = std::sqrt(sqErrFixed / static_cast<double>(T));

    std::cout << "Adaptive fusion RMSE : " << rmseAdaptive << "\n";
    std::cout << "Fixed-R fusion RMSE  : " << rmseFixed << "\n";
    std::cout << "Mean camera NIS      : " << nisCamMean << "\n";
    std::cout << "Mean LiDAR NIS       : " << nisLidarMean << "\n";
    return 0;
}
