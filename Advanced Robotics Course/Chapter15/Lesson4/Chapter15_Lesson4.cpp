#include <iostream>
#include <vector>
#include <random>

struct SwarmConfig {
    int N;
    int steps;
    double p_fail;
    double p_drop;
    double eps;
    double noise_std;
};

int main() {
    SwarmConfig cfg{50, 200, 0.2, 0.1, 0.2, 0.01};

    std::mt19937 gen(0);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::normal_distribution<double> gauss(0.0, cfg.noise_std);

    std::vector<double> x(cfg.N);
    std::vector<bool> alive(cfg.N, true);

    for (int i = 0; i < cfg.N; ++i) {
        x[i] = uni(gen);
        if (uni01(gen) < cfg.p_fail) {
            alive[i] = false;
        }
    }

    for (int k = 0; k < cfg.steps; ++k) {
        std::vector<double> x_new = x;
        for (int i = 0; i < cfg.N; ++i) {
            if (!alive[i]) continue;

            int left = (i - 1 + cfg.N) % cfg.N;
            int right = (i + 1) % cfg.N;

            bool link_left_ok = (uni01(gen) >= cfg.p_drop);
            bool link_right_ok = (uni01(gen) >= cfg.p_drop);

            double sum_diff = 0.0;
            int count = 0;

            if (link_left_ok && alive[left]) {
                sum_diff += (x[left] - x[i]);
                ++count;
            }
            if (link_right_ok && alive[right]) {
                sum_diff += (x[right] - x[i]);
                ++count;
            }

            if (count > 0) {
                double avg_diff = sum_diff / static_cast<double>(count);
                x_new[i] = x[i] + cfg.eps * avg_diff + gauss(gen);
            }
        }
        x.swap(x_new);
    }

    double sum = 0.0;
    int count_alive = 0;
    for (int i = 0; i < cfg.N; ++i) {
        if (alive[i]) {
            sum += x[i];
            ++count_alive;
        }
    }
    double mean = (count_alive > 0) ? sum / count_alive : 0.0;
    double err2 = 0.0;
    for (int i = 0; i < cfg.N; ++i) {
        if (alive[i]) {
            double d = x[i] - mean;
            err2 += d * d;
        }
    }
    double rmse = (count_alive > 0)
        ? std::sqrt(err2 / static_cast<double>(count_alive))
        : 0.0;

    std::cout << "Final RMSE among alive robots: " << rmse << std::endl;
    return 0;
}
      
