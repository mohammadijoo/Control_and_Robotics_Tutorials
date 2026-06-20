\
/*
Chapter 8 - Lesson 2: Importance Sampling and Resampling (Particle Filters)

C++ implementation (from scratch, STL only).

Robotics-oriented libraries where you will see these ideas:
  - ROS 1/2 (AMCL implementations, nav2)
  - MRPT (Mobile Robot Programming Toolkit)
  - Eigen (for linear algebra), though this file does not require it

Build (example):
  g++ -O2 -std=c++17 Chapter8_Lesson2.cpp -o pf_resample
Run:
  ./pf_resample
*/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

static double log_sum_exp(const std::vector<double>& logw) {
    double m = *std::max_element(logw.begin(), logw.end());
    double s = 0.0;
    for (double lw : logw) s += std::exp(lw - m);
    return m + std::log(s);
}

static std::vector<double> normalize_log_weights(const std::vector<double>& logw) {
    const double lse = log_sum_exp(logw);
    std::vector<double> w(logw.size());
    double sumw = 0.0;
    for (size_t i = 0; i < logw.size(); ++i) {
        w[i] = std::exp(logw[i] - lse);
        sumw += w[i];
    }
    for (double& wi : w) wi /= sumw;
    return w;
}

static double effective_sample_size(const std::vector<double>& w) {
    double s2 = 0.0;
    for (double wi : w) s2 += wi * wi;
    return 1.0 / s2;
}

static std::vector<int> systematic_resample(const std::vector<double>& w, std::mt19937& gen) {
    const int N = static_cast<int>(w.size());
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    const double u0 = unif(gen) / N;

    std::vector<double> cdf(N);
    std::partial_sum(w.begin(), w.end(), cdf.begin());

    std::vector<int> a(N);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        double u = u0 + static_cast<double>(i) / N;
        while (u > cdf[j] && j < N - 1) j++;
        a[i] = j;
    }
    return a;
}

static void demo_1d_tracking(unsigned seed = 7) {
    std::mt19937 gen(seed);
    std::normal_distribution<double> n01(0.0, 1.0);

    const int N = 2000;
    const double q = 0.4; // process std
    const double r = 0.7; // measurement std
    const double u = 0.3;

    std::vector<double> x(N);
    for (int i = 0; i < N; ++i) x[i] = 4.0 * n01(gen);

    double x_true = 2.0;
    for (int t = 1; t <= 8; ++t) {
        x_true = x_true + u + q * n01(gen);

        for (int i = 0; i < N; ++i) x[i] = x[i] + u + q * n01(gen);

        const double z = x_true + r * n01(gen);

        std::vector<double> logw(N);
        for (int i = 0; i < N; ++i) {
            const double e = (z - x[i]) / r;
            logw[i] = -0.5 * e * e; // ignore constants
        }
        std::vector<double> w = normalize_log_weights(logw);

        const double Neff = effective_sample_size(w);
        double x_hat = 0.0;
        for (int i = 0; i < N; ++i) x_hat += w[i] * x[i];

        std::cout << "t=" << t
                  << ", z=" << z
                  << ", true=" << x_true
                  << ", est=" << x_hat
                  << ", ESS=" << Neff << "\n";

        if (Neff < 0.5 * N) {
            std::vector<int> a = systematic_resample(w, gen);
            std::vector<double> x_new(N);
            for (int i = 0; i < N; ++i) x_new[i] = x[a[i]];
            x.swap(x_new);
        }
    }
}

int main() {
    demo_1d_tracking();
    return 0;
}
