#include <iostream>
#include <vector>
#include <random>
#include <cmath>

struct Metrics {
    double J;
    bool success;
    bool violation;
};

struct SimState {
    std::vector<double> state;
    std::vector<double> sensor;
    std::vector<double> goal;
    bool done;
    bool collision;
    bool task_success;
    double cost;
};

class Simulator {
public:
    int state_dim;

    virtual SimState reset(unsigned int noise_seed) = 0;
    virtual SimState step(const std::vector<double>& u) = 0;
    virtual ~Simulator() = default;
};

class Perception {
public:
    virtual void reset() = 0;
    virtual std::vector<double> update(const std::vector<double>& sensor) = 0;
    virtual ~Perception() = default;
};

class Planner {
public:
    virtual std::vector<double> plan(const std::vector<double>& belief,
                                        const std::vector<double>& goal) = 0;
    virtual ~Planner() = default;
};

class Controller {
public:
    virtual void reset() = 0;
    virtual std::vector<double> computeControl(const std::vector<double>& x_ref,
                                                  const std::vector<double>& belief) = 0;
    virtual ~Controller() = default;
};

Metrics runEpisode(Simulator& sim,
                   Perception& perc,
                   Planner& planner,
                   Controller& ctrl,
                   unsigned int noise_seed,
                   int horizon) {
    SimState sim_state = sim.reset(noise_seed);
    perc.reset();
    ctrl.reset();

    double J = 0.0;
    bool violated = false;
    bool success = false;

    for (int t = 0; t < horizon; ++t) {
        std::vector<double> belief = perc.update(sim_state.sensor);
        std::vector<double> x_ref = planner.plan(belief, sim_state.goal);
        std::vector<double> u = ctrl.computeControl(x_ref, belief);
        sim_state = sim.step(u);

        J += sim_state.cost;
        if (sim_state.collision) {
            violated = true;
        }
        if (sim_state.done) {
            success = !violated && sim_state.task_success;
            break;
        }
    }

    Metrics m;
    m.J = J;
    m.success = success;
    m.violation = violated;
    return m;
}

struct EvalResults {
    double J_hat;
    double J_std_over_sqrtN;
    double p_succ_hat;
    double p_succ_std_over_sqrtN;
    double p_viol_hat;
    double p_viol_std_over_sqrtN;
};

EvalResults evaluateStack(Simulator& sim,
                          Perception& perc,
                          Planner& planner,
                          Controller& ctrl,
                          int N,
                          int horizon) {
    std::mt19937 rng(0);
    std::uniform_int_distribution<unsigned int> seed_dist(0, 0xffffffffu);

    std::vector<double> Js;
    std::vector<double> successes;
    std::vector<double> violations;
    Js.reserve(N);
    successes.reserve(N);
    violations.reserve(N);

    for (int i = 0; i < N; ++i) {
        unsigned int seed = seed_dist(rng);
        Metrics m = runEpisode(sim, perc, planner, ctrl, seed, horizon);
        Js.push_back(m.J);
        successes.push_back(m.success ? 1.0 : 0.0);
        violations.push_back(m.violation ? 1.0 : 0.0);
    }

    auto mean_std_over_sqrtN = [N](const std::vector<double>& xs) {
        double mean = 0.0;
        for (double x : xs) mean += x;
        mean /= static_cast<double>(N);

        double var = 0.0;
        for (double x : xs) {
            double d = x - mean;
            var += d * d;
        }
        var /= static_cast<double>(N - 1);
        double std_over_sqrtN = std::sqrt(var / static_cast<double>(N));
        return std::make_pair(mean, std_over_sqrtN);
    };

    auto J_stats = mean_std_over_sqrtN(Js);
    auto s_stats = mean_std_over_sqrtN(successes);
    auto v_stats = mean_std_over_sqrtN(violations);

    EvalResults r;
    r.J_hat = J_stats.first;
    r.J_std_over_sqrtN = J_stats.second;
    r.p_succ_hat = s_stats.first;
    r.p_succ_std_over_sqrtN = s_stats.second;
    r.p_viol_hat = v_stats.first;
    r.p_viol_std_over_sqrtN = v_stats.second;
    return r;
}

int main() {
    // Instantiate concrete Simulator, Perception, Planner, Controller here,
    // then call evaluateStack and compare with mid-review thresholds.
    return 0;
}
      
