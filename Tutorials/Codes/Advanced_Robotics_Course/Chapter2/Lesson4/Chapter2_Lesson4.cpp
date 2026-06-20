#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <limits>

struct StateHash {
    std::size_t operator()(const State& s) const noexcept {
        // Provide a proper hash for your State type
        return s.hash();
    }
};

struct StateEqual {
    bool operator()(const State& a, const State& b) const noexcept {
        return a == b;
    }
};

struct Node {
    State s;
    double f;
    double g;
};

struct NodeCompare {
    bool operator()(const Node& a, const Node& b) const noexcept {
        // priority_queue in C++ is max-heap by default, so invert comparison
        return a.f > b.f;
    }
};

std::vector<State> reconstruct_path(
    const State& goal,
    const std::unordered_map<State, State, StateHash, StateEqual>& parent)
{
    std::vector<State> path;
    State cur = goal;
    auto it = parent.find(cur);
    while (it != parent.end()) {
        path.push_back(cur);
        auto it2 = parent.find(cur);
        if (it2 == parent.end() || it2->second == cur) {
            break;
        }
        cur = it2->second;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

template <typename SuccessorFunc, typename HeuristicFunc, typename GoalPredicate>
bool astar(
    const State& start,
    GoalPredicate is_goal,
    SuccessorFunc successors,
    HeuristicFunc heuristic,
    std::vector<State>& out_path)
{
    using Map = std::unordered_map<State, double, StateHash, StateEqual>;
    using ParentMap = std::unordered_map<State, State, StateHash, StateEqual>;

    Map g;
    ParentMap parent;
    g[start] = 0.0;
    parent[start] = start;

    std::priority_queue<Node, std::vector<Node>, NodeCompare> open;
    open.push(Node{start, heuristic(start), 0.0});
    std::unordered_set<State, StateHash, StateEqual> closed;

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (closed.find(cur.s) != closed.end()) {
            continue;
        }
        closed.insert(cur.s);

        if (is_goal(cur.s)) {
            out_path = reconstruct_path(cur.s, parent);
            return true;
        }

        double g_s = g[cur.s];
        for (const auto& [s_next, cost] : successors(cur.s)) {
            if (cost < 0.0) {
                throw std::runtime_error("Negative edge cost not allowed");
            }
            double new_g = g_s + cost;
            auto it = g.find(s_next);
            if (it == g.end() || new_g < it->second) {
                g[s_next] = new_g;
                parent[s_next] = cur.s;
                double f_next = new_g + heuristic(s_next);
                open.push(Node{s_next, f_next, new_g});
            }
        }
    }
    return false; // no solution
}
      
