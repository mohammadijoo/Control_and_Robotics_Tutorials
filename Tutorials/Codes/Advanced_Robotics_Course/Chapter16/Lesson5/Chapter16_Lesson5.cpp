#include <vector>
#include <iostream>
#include <limits>

struct Transition {
    int action_id;
    std::vector<int> succ;  // successors for this (state, action)
};

int main() {
    int n_states = 10;
    int n_actions = 4;

    // For each state, a list of (action, successors)
    std::vector<std::vector<Transition>> delta(n_states);

    // Example: fill delta[s] with transitions (application-specific)
    // ...

    std::vector<bool> safe(n_states, true);
    std::vector<bool> goal(n_states, false);

    // Mark some states as unsafe / goal
    // safe[i] = false for obstacles, goal[i] = true for goals

    // W^0 = Safe ∩ Goal
    std::vector<bool> W(n_states, false);
    for (int s = 0; s < n_states; ++s) {
        if (safe[s] && goal[s]) {
            W[s] = true;
        }
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int s = 0; s < n_states; ++s) {
            if (!safe[s] || W[s]) continue;  // skip unsafe or already winning
            bool can_control = false;
            for (const auto &tr : delta[s]) {
                if (tr.succ.empty()) continue;
                bool all_in_W = true;
                for (int sp : tr.succ) {
                    if (!W[sp]) {
                        all_in_W = false;
                        break;
                    }
                }
                if (all_in_W) {
                    can_control = true;
                    break;
                }
            }
            if (can_control) {
                W[s] = true;
                changed = true;
            }
        }
    }

    // Extract a memoryless strategy: for each winning state, choose one action
    std::vector<int> strategy(n_states, -1);
    for (int s = 0; s < n_states; ++s) {
        if (!W[s]) continue;
        for (const auto &tr : delta[s]) {
            if (tr.succ.empty()) continue;
            bool all_in_W = true;
            for (int sp : tr.succ) {
                if (!W[sp]) {
                    all_in_W = false;
                    break;
                }
            }
            if (all_in_W) {
                strategy[s] = tr.action_id;
                break;
            }
        }
    }

    int start_state = 0;
    std::cout << "Is start winning? " << (W[start_state] ? "yes" : "no") << std::endl;
    std::cout << "Strategy at start: " << strategy[start_state] << std::endl;

    return 0;
}
      
