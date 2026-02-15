#include <vector>
#include <cmath>
#include <numeric>

using Belief = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

// T[a][s][s_next]
using TransitionModel = std::vector<std::vector<std::vector<double>>>;
// Z[a][s_next][o]
using ObservationModel = std::vector<std::vector<std::vector<double>>>;

Belief predictBelief(const Belief& b,
                     const TransitionModel& T,
                     int a) {
    const int S = static_cast<int>(b.size());
    Belief bp(S, 0.0);
    for (int s = 0; s < S; ++s) {
        for (int sp = 0; sp < S; ++sp) {
            bp[sp] += T[a][s][sp] * b[s];
        }
    }
    return bp;
}

Belief updateBelief(const Belief& b,
                    const TransitionModel& T,
                    const ObservationModel& Z,
                    int a, int o) {
    Belief bp = predictBelief(b, T, a);
    const int S = static_cast<int>(b.size());
    Belief bnew(S, 0.0);
    double norm = 0.0;
    for (int sp = 0; sp < S; ++sp) {
        bnew[sp] = Z[a][sp][o] * bp[sp];
        norm += bnew[sp];
    }
    if (norm < 1e-12) {
        // fallback: uniform
        std::fill(bnew.begin(), bnew.end(), 1.0 / S);
        return bnew;
    }
    for (int sp = 0; sp < S; ++sp) {
        bnew[sp] /= norm;
    }
    return bnew;
}

// Example: inside a ROS node controlling a robot arm that manipulates a rope,
// you would call updateBelief after each observation and use an OMPL-based
// planner that chooses actions based on the current belief.
      
