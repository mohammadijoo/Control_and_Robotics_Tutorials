#include <random>
#include <vector>
#include <iostream>

struct Params {
    double mass_scale;
    double friction_scale;
    double latency;
};

struct EpisodeStats {
    double ret;
    bool success;
};

class RandomizationConfig {
public:
    RandomizationConfig(double m_min, double m_max,
                        double f_min, double f_max,
                        double l_min, double l_max)
        : mass_min_(m_min), mass_max_(m_max),
          fric_min_(f_min), fric_max_(f_max),
          lat_min_(l_min), lat_max_(l_max) {}

    Params sample(std::mt19937& gen) const {
        std::uniform_real_distribution<double> mass_dist(mass_min_, mass_max_);
        std::uniform_real_distribution<double> fric_dist(fric_min_, fric_max_);
        std::uniform_real_distribution<double> lat_dist(lat_min_, lat_max_);
        Params p;
        p.mass_scale = mass_dist(gen);
        p.friction_scale = fric_dist(gen);
        p.latency = lat_dist(gen);
        return p;
    }

private:
    double mass_min_, mass_max_;
    double fric_min_, fric_max_;
    double lat_min_, lat_max_;
};

EpisodeStats run_episode_sim(RandomizedArmEnv& env,
                             Policy& policy,
                             const Params& params,
                             int max_steps) {
    env.reset(params);
    double total_reward = 0.0;
    bool success = false;
    for (int t = 0; t < max_steps; ++t) {
        Eigen::VectorXd obs = env.observe();
        Eigen::VectorXd action = policy.act(obs);
        StepResult step = env.step(action);
        total_reward += step.reward;
        if (step.info.is_success) {
            success = true;
        }
        if (step.done) {
            break;
        }
    }
    return {total_reward, success};
}

// On real hardware, use an analogous function that calls low-level controllers.
EpisodeStats run_episode_real(Policy& policy,
                              const Params& params,
                              int max_steps);

int main() {
    RandomizedArmEnv env;
    Policy policy = load_trained_policy("policy.bin");

    RandomizationConfig eval_cfg(0.9, 1.1,   // mass_range
                                 0.8, 1.2,   // friction_range
                                 0.0, 0.03); // latency_range

    std::mt19937 gen(42);
    const int N = 40;
    const int max_steps = 200;

    std::vector<double> sim_returns, real_returns;
    std::vector<int> sim_success, real_success;

    sim_returns.reserve(N);
    real_returns.reserve(N);
    sim_success.reserve(N);
    real_success.reserve(N);

    for (int i = 0; i < N; ++i) {
        Params p = eval_cfg.sample(gen);
        EpisodeStats es_sim = run_episode_sim(env, policy, p, max_steps);
        EpisodeStats es_real = run_episode_real(policy, p, max_steps);

        sim_returns.push_back(es_sim.ret);
        real_returns.push_back(es_real.ret);
        sim_success.push_back(es_sim.success ? 1 : 0);
        real_success.push_back(es_real.success ? 1 : 0);
    }

    auto mean = [](const std::vector<double>& v) {
        double s = 0.0;
        for (double x : v) s += x;
        return s / static_cast<double>(v.size());
    };

    auto mean_int = [](const std::vector<int>& v) {
        double s = 0.0;
        for (int x : v) s += static_cast<double>(x);
        return s / static_cast<double>(v.size());
    };

    double J_sim = mean(sim_returns);
    double J_real = mean(real_returns);
    double p_sim = mean_int(sim_success);
    double p_real = mean_int(real_success);

    std::cout << "Expected return (sim, real): "
              << J_sim << ", " << J_real << std::endl;
    std::cout << "Success prob. (sim, real): "
              << p_sim << ", " << p_real << std::endl;
    std::cout << "Transfer gap (return): " << (J_real - J_sim) << std::endl;
    std::cout << "Transfer gap (success): " << (p_real - p_sim) << std::endl;

    return 0;
}
      
