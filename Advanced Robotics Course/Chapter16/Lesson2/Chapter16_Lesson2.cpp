#include <vector>
#include <string>
#include <unordered_set>

using State = std::string;
using AP = std::string;
using Label = std::unordered_set<AP>;

// Finite trace of labels
bool checkGloballyNotCollision(const std::vector<Label>& trace) {
    for (const auto& lab : trace) {
        if (lab.count("collision") != 0) {
            return false;
        }
    }
    return true;
}

// Example integration point: after generating a path in a planner
bool checkTask(const std::vector<Label>& trace) {
    // G !collision  and  F goal
    bool safety = checkGloballyNotCollision(trace);
    bool reachGoal = false;
    for (const auto& lab : trace) {
        if (lab.count("goal") != 0) {
            reachGoal = true;
            break;
        }
    }
    return safety && reachGoal;
}
      
