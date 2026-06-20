// Chapter17_Lesson5.cpp
#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iomanip>

struct State {
    double x;
    double v;
};

State dynamics(const State& s, double force, double k, double c, double m) {
    State ds;
    ds.x = s.v;
    ds.v = -(k / m) * s.x - (c / m) * s.v + force / m;
    return ds;
}

State rk4Step(const State& s, double force, double k, double c, double m, double h) {
    State k1 = dynamics(s, force, k, c, m);

    State s2{ s.x + 0.5 * h * k1.x, s.v + 0.5 * h * k1.v };
    State k2 = dynamics(s2, force, k, c, m);

    State s3{ s.x + 0.5 * h * k2.x, s.v + 0.5 * h * k2.v };
    State k3 = dynamics(s3, force, k, c, m);

    State s4{ s.x + h * k3.x, s.v + h * k3.v };
    State k4 = dynamics(s4, force, k, c, m);

    State out;
    out.x = s.x + (h / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    out.v = s.v + (h / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    return out;
}

int main() {
    const double T = 10.0;
    const double dt = 0.005;
    const int N = 2000;
    const double m = 1.0;

    const double kMean = 25.0, kStd = 2.0;
    const double cMean = 1.5, cStd = 0.2;
    const double x0Mean = 0.0, x0Std = 0.05;
    const double v0Mean = 0.0, v0Std = 0.05;
    const double sigmaF = 2.0;
    const double xThreshold = 0.75;

    const int nt = static_cast<int>(std::round(T / dt)) + 1;

    std::vector<double> t(nt), sumX(nt, 0.0), sumX2(nt, 0.0);
    for (int i = 0; i < nt; ++i) t[i] = i * dt;

    std::mt19937 rng(17);
    std::normal_distribution<double> distK(kMean, kStd);
    std::normal_distribution<double> distC(cMean, cStd);
    std::normal_distribution<double> distX0(x0Mean, x0Std);
    std::normal_distribution<double> distV0(v0Mean, v0Std);
    std::normal_distribution<double> distF(0.0, sigmaF);

    int exceedCount = 0;

    for (int trial = 0; trial < N; ++trial) {
        double k = std::max(1e-6, distK(rng));
        double c = std::max(1e-6, distC(rng));

        State s{ distX0(rng), distV0(rng) };

        std::vector<double> xHist(nt, 0.0);
        xHist[0] = s.x;

        double peakAbs = std::abs(s.x);

        for (int n = 0; n < nt - 1; ++n) {
            double F = distF(rng);
            s = rk4Step(s, F, k, c, m, dt);
            xHist[n + 1] = s.x;
            peakAbs = std::max(peakAbs, std::abs(s.x));
        }

        for (int n = 0; n < nt; ++n) {
            sumX[n] += xHist[n];
            sumX2[n] += xHist[n] * xHist[n];
        }

        if (peakAbs > xThreshold) {
            exceedCount++;
        }
    }

    std::vector<double> meanX(nt), varX(nt), stdX(nt);
    for (int n = 0; n < nt; ++n) {
        meanX[n] = sumX[n] / N;
        varX[n] = (sumX2[n] - N * meanX[n] * meanX[n]) / (N - 1.0);
        if (varX[n] < 0.0) varX[n] = 0.0;
        stdX[n] = std::sqrt(varX[n]);
    }

    const double z975 = 1.959963984540054;
    double seFinal = stdX.back() / std::sqrt(static_cast<double>(N));
    double ciMeanLo = meanX.back() - z975 * seFinal;
    double ciMeanHi = meanX.back() + z975 * seFinal;

    double pHat = static_cast<double>(exceedCount) / N;
    double seP = std::sqrt(std::max(pHat * (1.0 - pHat), 1e-12) / N);
    double ciPLo = std::max(0.0, pHat - z975 * seP);
    double ciPHi = std::min(1.0, pHat + z975 * seP);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Monte Carlo trajectories: " << N << "\n";
    std::cout << "Final mean x(T): " << meanX.back() << "\n";
    std::cout << "Final variance x(T): " << varX.back() << "\n";
    std::cout << "95% CI for E[x(T)]: [" << ciMeanLo << ", " << ciMeanHi << "]\n";
    std::cout << "P(max |x| > " << xThreshold << "): " << pHat << "\n";
    std::cout << "95% CI for probability: [" << ciPLo << ", " << ciPHi << "]\n";

    std::ofstream csv("Chapter17_Lesson5_cpp_results.csv");
    csv << "t,mean_x,var_x,std_x\n";
    for (int n = 0; n < nt; ++n) {
        csv << t[n] << "," << meanX[n] << "," << varX[n] << "," << stdX[n] << "\n";
    }
    csv.close();

    return 0;
}
