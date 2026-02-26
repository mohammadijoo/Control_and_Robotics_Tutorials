// Chapter20_Lesson4.cpp
// Navigation Stack Deployment (Capstone AMR)
// C++17 educational deployment manager + watchdog + inflation utility.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

enum class NodeState { Unconfigured, Inactive, Active, Error };

struct ManagedNode {
    std::string name;
    double startup_ms;
    NodeState state{NodeState::Unconfigured};

    bool configure() {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(startup_ms)));
        state = NodeState::Inactive;
        return true;
    }
    bool activate() {
        if (state != NodeState::Inactive) {
            state = NodeState::Error;
            return false;
        }
        state = NodeState::Active;
        return true;
    }
    void deactivate() {
        if (state == NodeState::Active) state = NodeState::Inactive;
    }
    void reset() { state = NodeState::Unconfigured; }
};

struct TimingBudget {
    double period_ms{100.0};
    std::unordered_map<std::string, double> wcet_ms;

    double utilization() const {
        double sum = 0.0;
        for (const auto& kv : wcet_ms) sum += kv.second / period_ms;
        return sum;
    }

    bool schedulable() const { return utilization() <= 1.0; }

    void printReport() const {
        double total = 0.0;
        for (const auto& kv : wcet_ms) total += kv.second;
        std::cout << "Period=" << period_ms << " ms, Total WCET=" << total
                  << " ms, U=" << std::fixed << std::setprecision(3)
                  << utilization() << ", schedulable=" << std::boolalpha
                  << schedulable() << "\n";
    }
};

struct CostmapInflation {
    double r_ins{0.22};
    double r_inf{0.70};
    int c_lethal{254};
    int c_ins{220};
    double kappa{8.0};

    int cost(double d) const {
        if (d <= r_ins) return c_lethal;
        if (d >= r_inf) return 0;
        double v = c_ins * std::exp(-kappa * (d - r_ins));
        v = std::clamp(v, 1.0, static_cast<double>(c_ins));
        return static_cast<int>(std::round(v));
    }
};

struct TrajectoryScoreInput {
    double dist_to_path;
    double dist_to_goal;
    int obstacle_cost;
    double v;
    double omega;
};

struct PlannerWeights {
    double w_path{1.4};
    double w_goal{1.0};
    double w_obs{2.6};
    double w_vel{0.4};
    double w_spin{0.2};
};

double scoreTrajectory(const TrajectoryScoreInput& x, double v_ref, const PlannerWeights& w) {
    double obs_term = static_cast<double>(x.obstacle_cost) / 254.0;
    return w.w_path * x.dist_to_path +
           w.w_goal * x.dist_to_goal +
           w.w_obs * obs_term +
           w.w_vel * std::abs(x.v - v_ref) +
           w.w_spin * std::abs(x.omega);
}

class DeploymentSupervisor {
public:
    DeploymentSupervisor(std::vector<ManagedNode> nodes, TimingBudget timing)
        : nodes_(std::move(nodes)), timing_(std::move(timing)) {}

    bool startup() {
        if (!timing_.schedulable()) {
            std::cout << "[startup] timing budget invalid\n";
            return false;
        }
        for (auto& n : nodes_) {
            if (!n.configure()) return false;
        }
        for (auto& n : nodes_) {
            if (!n.activate()) return false;
        }
        active_ = true;
        std::cout << "[startup] all nodes active\n";
        return true;
    }

    bool localizationHealthy(double cov_xx, double cov_yy) const {
        double trace_xy = cov_xx + cov_yy;
        return trace_xy <= cov_trace_limit_;
    }

    bool plannerDeadlineCheck(double runtime_ms) {
        if (runtime_ms > planner_deadline_ms_) {
            planner_miss_count_++;
        } else {
            planner_miss_count_ = 0;
        }
        if (planner_miss_count_ >= max_misses_) {
            std::cout << "[recovery] planner misses threshold reached\n";
            planner_miss_count_ = 0;
            return false;
        }
        return true;
    }

    void shutdown() {
        for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
            it->deactivate();
            it->reset();
        }
        active_ = false;
        std::cout << "[shutdown] nodes reset\n";
    }

private:
    std::vector<ManagedNode> nodes_;
    TimingBudget timing_;
    bool active_{false};
    double cov_trace_limit_{0.20 * 0.20};
    int planner_miss_count_{0};
    int max_misses_{3};
    double planner_deadline_ms_{80.0};
};

int main() {
    TimingBudget budget;
    budget.period_ms = 100.0;
    budget.wcet_ms = {
        {"sensor_preprocess", 8.0},
        {"localization", 14.0},
        {"costmap_update", 18.0},
        {"global_planner", 12.0},
        {"local_planner", 22.0},
        {"controller", 5.0},
        {"bt_tick", 3.0}
    };
    budget.printReport();

    std::vector<ManagedNode> nodes = {
        {"map_server", 20.0},
        {"localization", 25.0},
        {"global_planner", 15.0},
        {"local_planner", 12.0},
        {"controller_server", 10.0},
        {"bt_navigator", 18.0}
    };

    DeploymentSupervisor sup(nodes, budget);
    if (!sup.startup()) return 1;

    CostmapInflation infl;
    std::cout << "Inflation profile:\n";
    for (double d : {0.18, 0.25, 0.35, 0.50, 0.80}) {
        std::cout << "d=" << d << " -> " << infl.cost(d) << "\n";
    }

    PlannerWeights w;
    std::vector<TrajectoryScoreInput> candidates = {
        {0.05, 1.20, 140, 0.40, 0.10},
        {0.18, 0.95,  80, 0.55, 0.35},
        {0.10, 1.00, 220, 0.50, 0.05}
    };
    int best_idx = -1;
    double best_score = 1e9;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        double s = scoreTrajectory(candidates[i], 0.50, w);
        std::cout << "Candidate " << i << " score=" << s << "\n";
        if (s < best_score) { best_score = s; best_idx = i; }
    }
    std::cout << "Best candidate=" << best_idx << "\n";

    std::vector<double> planner_runtimes = {60, 72, 91, 88, 95, 65, 77, 83, 90};
    std::vector<std::pair<double,double>> covariances = {
        {0.006,0.007}, {0.007,0.006}, {0.008,0.005},
        {0.010,0.009}, {0.012,0.011}, {0.035,0.034}, // degraded localization
        {0.009,0.007}, {0.008,0.008}, {0.007,0.006}
    };

    for (std::size_t k = 0; k < planner_runtimes.size(); ++k) {
        bool loc_ok = sup.localizationHealthy(covariances[k].first, covariances[k].second);
        if (!loc_ok) {
            std::cout << "[recovery] localization covariance too large, slow/relocalize\n";
        }
        (void) sup.plannerDeadlineCheck(planner_runtimes[k]);
    }

    sup.shutdown();
    return 0;
}
