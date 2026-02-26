/*
Chapter6_Lesson2.cpp
Bayes Filters for Mobile Robots (discrete-state version)

Implements:
  bel_bar[i] = sum_j T[i][j] * bel_prev[j]
  bel[i]     = eta * L[i] * bel_bar[i]

Compile:
  g++ -O2 -std=c++17 Chapter6_Lesson2.cpp -o bayes_filter
*/

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

static std::vector<double> normalize(const std::vector<double>& p, double eps = 1e-12) {
    double s = std::accumulate(p.begin(), p.end(), 0.0);
    std::vector<double> out(p.size(), 0.0);
    if (s < eps) {
        double u = 1.0 / static_cast<double>(p.size());
        std::fill(out.begin(), out.end(), u);
        return out;
    }
    for (size_t i = 0; i < p.size(); ++i) out[i] = p[i] / s;
    return out;
}

static std::vector<double> predict(const std::vector<double>& bel_prev,
                                   const std::vector<std::vector<double>>& T) {
    const size_t N = bel_prev.size();
    if (T.size() != N || T[0].size() != N) throw std::runtime_error("T must be NxN");
    std::vector<double> bel_bar(N, 0.0);
    for (size_t i = 0; i < N; ++i) {
        double s = 0.0;
        for (size_t j = 0; j < N; ++j) s += T[i][j] * bel_prev[j];
        bel_bar[i] = s;
    }
    return normalize(bel_bar);
}

static std::vector<double> update(const std::vector<double>& bel_bar,
                                  const std::vector<double>& likelihood) {
    const size_t N = bel_bar.size();
    if (likelihood.size() != N) throw std::runtime_error("likelihood size mismatch");
    std::vector<double> bel(N, 0.0);
    for (size_t i = 0; i < N; ++i) bel[i] = likelihood[i] * bel_bar[i];
    return normalize(bel);
}

static std::vector<std::vector<double>> make_shift_transition(size_t N, int delta,
                                                              double p_exact = 0.8,
                                                              double p_under = 0.1,
                                                              double p_over = 0.1) {
    if (std::abs((p_exact + p_under + p_over) - 1.0) > 1e-9) throw std::runtime_error("probabilities must sum to 1");
    std::vector<std::vector<double>> T(N, std::vector<double>(N, 0.0));
    for (size_t j = 0; j < N; ++j) {
        auto modN = [N](long long k) -> size_t {
            long long r = k % static_cast<long long>(N);
            if (r < 0) r += static_cast<long long>(N);
            return static_cast<size_t>(r);
        };
        size_t i_exact = modN(static_cast<long long>(j) + delta);
        size_t i_under = modN(static_cast<long long>(j) + delta - 1);
        size_t i_over  = modN(static_cast<long long>(j) + delta + 1);
        T[i_exact][j] += p_exact;
        T[i_under][j] += p_under;
        T[i_over][j]  += p_over;
    }
    return T;
}

static std::vector<double> landmark_likelihood(size_t N, int z,
                                               const std::vector<int>& landmarks,
                                               double p_hit = 0.9,
                                               double p_false = 0.1) {
    std::vector<double> L(N, (z == 1) ? p_false : (1.0 - p_false));
    for (int lm : landmarks) {
        size_t x = static_cast<size_t>(((lm % static_cast<int>(N)) + static_cast<int>(N)) % static_cast<int>(N));
        L[x] = (z == 1) ? p_hit : (1.0 - p_hit);
    }
    return L;
}

int main() {
    const size_t N = 20;
    std::vector<double> bel(N, 1.0 / static_cast<double>(N));
    std::vector<int> landmarks = {3, 14};
    std::vector<int> controls  = {1, 1, 1, 2, 1, 1, 2, 1};

    std::mt19937 rng(4);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    std::uniform_int_distribution<int> unif_state(0, static_cast<int>(N) - 1);

    int x = unif_state(rng);
    std::vector<int> truth = {x};
    std::vector<int> zs;

    // Simulate
    for (int u : controls) {
        double r = unif(rng);
        if (r < 0.8) x = (x + u) % static_cast<int>(N);
        else if (r < 0.9) x = (x + u - 1 + static_cast<int>(N)) % static_cast<int>(N);
        else x = (x + u + 1) % static_cast<int>(N);
        truth.push_back(x);

        bool at_landmark = (x == landmarks[0] || x == landmarks[1]);
        int z = 0;
        if (at_landmark) z = (unif(rng) < 0.9) ? 1 : 0;
        else z = (unif(rng) < 0.1) ? 1 : 0;
        zs.push_back(z);
    }

    // Filter
    for (size_t t = 0; t < controls.size(); ++t) {
        int u = controls[t];
        int z = zs[t];
        auto T = make_shift_transition(N, u);
        auto L = landmark_likelihood(N, z, landmarks);
        auto bel_bar = predict(bel, T);
        bel = update(bel_bar, L);
    }

    // Report top-5
    std::vector<size_t> idx(N);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b){ return bel[a] > bel[b]; });

    std::cout << "Final belief (top 5 states):\n";
    for (int k = 0; k < 5; ++k) {
        size_t i = idx[k];
        std::cout << "  state " << std::setw(2) << i << ": " << std::fixed << std::setprecision(4) << bel[i] << "\n";
    }
    std::cout << "Truth trajectory: ";
    for (size_t i = 0; i < truth.size(); ++i) {
        std::cout << truth[i] << (i + 1 < truth.size() ? ", " : "\n");
    }
    return 0;
}
