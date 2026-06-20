#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>

using Object = std::string;
using Fact   = std::pair<std::string, std::vector<Object>>;
using Binding = std::unordered_map<std::string, Object>;

struct Stream {
    std::string name;
    std::function<bool(const Binding&, const std::vector<Fact>&)> precond;
    std::function<bool(const Binding&, Binding&)> sampler;
    std::function<std::vector<Fact>(const Binding&)> head;

    bool call(const Binding& b,
              const std::vector<Fact>& facts,
              std::vector<Fact>& out_facts) const {
        if (!precond(b, facts)) return false;
        Binding sample;
        if (!sampler(b, sample)) return false;
        out_facts = head(sample);
        return !out_facts.empty();
    }
};

// Example motion stream that would internally call OMPL or MoveIt
bool motion_sampler(const Binding& b, Binding& out) {
    Object q_start = b.at("q_start");
    Object q_goal  = b.at("q_goal");
    // TODO: encode q_start, q_goal as OMPL states and call a planner.
    // Here we simulate success.
    out["traj"] = "traj_" + q_start + "_" + q_goal;
    return true;
}

std::vector<Fact> motion_head(const Binding& b) {
    return { Fact{"reachable", {b.at("q_start"), b.at("q_goal")}} };
}

int main() {
    Stream motion_stream{
        "sample-motion",
        [](const Binding&, const std::vector<Fact>&) { return true; },
        motion_sampler,
        motion_head
    };

    std::vector<Fact> facts;
    Binding b;
    b["q_start"] = "q0";
    b["q_goal"]  = "q1";
    std::vector<Fact> new_facts;
    if (motion_stream.call(b, facts, new_facts)) {
        std::cout << "Stream succeeded, new facts:\n";
        for (auto& f : new_facts) {
            std::cout << f.first << " ";
            for (auto& o : f.second) std::cout << o << " ";
            std::cout << "\n";
        }
    }
    return 0;
}
      
