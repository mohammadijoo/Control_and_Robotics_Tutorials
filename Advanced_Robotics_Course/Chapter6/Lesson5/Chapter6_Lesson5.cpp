#include <iostream>
#include <vector>
#include <cmath>

struct RobustMDP {
    int nStates;
    int nActions;
    double gamma;
    // Costs: c[s][a]
    std::vector<std::vector<double> > cost;
    // Interval uncertainty: pMin[s][a][s'], pMax[s][a][s']
    std::vector<std::vector<std::vector<double> > > pMin, pMax;

    RobustMDP(int nS, int nA, double g)
        : nStates(nS), nActions(nA), gamma(g) {
        cost.assign(nStates, std::vector<double>(nActions, 0.0));
        pMin.assign(nStates,
                    std::vector<std::vector<double> >(
                        nActions, std::vector<double>(nStates, 0.0)));
        pMax = pMin;
    }

    double innerWorst(const std::vector<double> &V, int s, int a) const {
        // Greedy saturating allocation
        std::vector<double> v = V;
        std::vector<int> order(nStates);
        for (int i = 0; i < nStates; ++i) order[i] = i;
        std::sort(order.begin(), order.end(),
                  [&](int i, int j) { return v[i] > v[j]; });

        std::vector<double> p(nStates, 0.0);
        double remaining = 1.0;
        for (int idx = 0; idx < nStates; ++idx) {
            int sp = order[idx];
            double maxAllow = std::min(pMax[s][a][sp], remaining);
            double minAllow = pMin[s][a][sp];
            double alloc = std::max(minAllow, maxAllow);
            alloc = std::min(alloc, remaining);
            p[sp] = alloc;
            remaining -= alloc;
            if (remaining <= 1e-12) break;
        }
        if (remaining > 1e-12) {
            // distribute leftover uniformly among non-saturated states
            std::vector<int> freeIdx;
            for (int sp = 0; sp < nStates; ++sp) {
                if (p[sp] < pMax[s][a][sp] - 1e-12) {
                    freeIdx.push_back(sp);
                }
            }
            if (!freeIdx.empty()) {
                double extra = remaining / double(freeIdx.size());
                for (int sp : freeIdx) {
                    p[sp] = std::min(pMax[s][a][sp], p[sp] + extra);
                }
            }
        }
        double val = 0.0;
        for (int sp = 0; sp < nStates; ++sp) {
            val += p[sp] * V[sp];
        }
        return val;
    }

    void valueIteration(std::vector<double> &V, std::vector<int> &policy,
                        int maxIter = 200, double tol = 1e-6) const {
        V.assign(nStates, 0.0);
        std::vector<double> Vnew(nStates);
        for (int it = 0; it < maxIter; ++it) {
            double diff = 0.0;
            for (int s = 0; s < nStates; ++s) {
                double best = 1e300;
                for (int a = 0; a < nActions; ++a) {
                    double worst = innerWorst(V, s, a);
                    double q = cost[s][a] + gamma * worst;
                    if (q < best) best = q;
                }
                Vnew[s] = best;
                diff = std::max(diff, std::fabs(Vnew[s] - V[s]));
            }
            V = Vnew;
            if (diff < tol) break;
        }
        policy.assign(nStates, 0);
        for (int s = 0; s < nStates; ++s) {
            double best = 1e300;
            int bestA = 0;
            for (int a = 0; a < nActions; ++a) {
                double worst = innerWorst(V, s, a);
                double q = cost[s][a] + gamma * worst;
                if (q < best) { best = q; bestA = a; }
            }
            policy[s] = bestA;
        }
    }
};

int main() {
    RobustMDP mdp(5, 2, 0.95);
    // Fill cost, pMin, pMax from robot motion primitive model...
    std::vector<double> V;
    std::vector<int> policy;
    mdp.valueIteration(V, policy);
    std::cout << "Robust value function:" << std::endl;
    for (double v : V) std::cout << v << " ";
    std::cout << std::endl;
    return 0;
}
      
