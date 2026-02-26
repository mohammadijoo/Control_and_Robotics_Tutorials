#include <vector>
#include <array>
#include <cmath>

struct Trajectory {
    std::vector<std::array<double, 7>> q;      // example: 7-DOF joint angles
    std::vector<std::array<double, 7>> qd;     // joint velocities
    std::vector<std::array<double, 3>> ee_pos; // end-effector position
};

struct Experiment {
    Trajectory real_traj;
    // plus inputs, initial state, etc.
};

class SimulatorInterface {
public:
    Trajectory rolloutSim(const std::vector<double>& theta,
                          const Experiment& exp) const;
};

std::vector<double> trajectoryFeatures(const Trajectory& traj) {
    // Concatenate mean and std of q, qd, ee_pos
    const std::size_t T = traj.q.size();
    const std::size_t nq = traj.q.front().size();
    const std::size_t ne = traj.ee_pos.front().size();

    std::vector<double> feats;
    feats.reserve(2 * (nq + nq + ne)); // mean and std for each coord

    auto accumulate_mean_std = [&](const auto& seq, std::size_t dim) {
        std::vector<double> mean(dim, 0.0);
        std::vector<double> sq(dim, 0.0);
        for (const auto& x : seq) {
            for (std::size_t i = 0; i < dim; ++i) {
                mean[i] += x[i];
                sq[i]   += x[i] * x[i];
            }
        }
        for (std::size_t i = 0; i < dim; ++i) {
            mean[i] /= static_cast<double>(T);
            double var = sq[i] / static_cast<double>(T) - mean[i] * mean[i];
            double stdv = std::sqrt(std::max(var, 0.0));
            feats.push_back(mean[i]);
            feats.push_back(stdv);
        }
    };

    accumulate_mean_std(traj.q, nq);
    accumulate_mean_std(traj.qd, nq);
    accumulate_mean_std(traj.ee_pos, ne);

    return feats;
}

double featureMismatch(const Trajectory& real_traj,
                       const Trajectory& sim_traj) {
    auto fr = trajectoryFeatures(real_traj);
    auto fs = trajectoryFeatures(sim_traj);
    double cost = 0.0;
    const std::size_t n = fr.size();
    for (std::size_t i = 0; i < n; ++i) {
        double d = fr[i] - fs[i];
        cost += d * d;
    }
    return cost;
}

double calibrationObjective(const std::vector<double>& theta,
                            const std::vector<Experiment>& exps,
                            const SimulatorInterface& sim) {
    double total = 0.0;
    for (const auto& e : exps) {
        Trajectory sim_traj = sim.rolloutSim(theta, e);
        total += featureMismatch(e.real_traj, sim_traj);
    }
    return total;
}
      
