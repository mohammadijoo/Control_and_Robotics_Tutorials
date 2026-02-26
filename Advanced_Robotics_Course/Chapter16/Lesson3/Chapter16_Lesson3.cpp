#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <utility>

struct State2D {
    int i, j;
    bool operator<(const State2D& other) const {
        if (i != other.i) return i < other.i;
        return j < other.j;
    }
    bool operator==(const State2D& other) const {
        return i == other.i && j == other.j;
    }
};

struct TS {
    std::set<State2D> states;
    State2D initial;
    std::vector<std::string> actions{"up", "down", "left", "right"};
    std::map<std::pair<State2D,std::string>, std::set<State2D> > trans;
    std::map<State2D, std::set<std::string> > label;
};

struct Buchi {
    // F goal
    std::set<std::string> states{"q0", "q1"};
    std::string initial{"q0"};
    std::set<std::string> accepting{"q1"};

    std::string delta(const std::string& q,
                      const std::set<std::string>& sigma) const {
        bool has_goal = sigma.find("goal") != sigma.end();
        if (q == "q0") {
            return has_goal ? "q1" : "q0";
        } else {
            return "q1";
        }
    }
};

using ProdState = std::pair<State2D,std::string>;

struct ProdStateLess {
    bool operator()(const ProdState& a, const ProdState& b) const {
        if (a.first < b.first) return true;
        if (b.first < a.first) return false;
        return a.second < b.second;
    }
};

std::vector<std::pair<ProdState,std::string> >
productSuccessors(const TS& ts, const Buchi& ba,
                  const ProdState& ps) {
    std::vector<std::pair<ProdState,std::string> > out;
    const State2D& s = ps.first;
    const std::string& q = ps.second;
    auto it_label = ts.label.find(s);
    const std::set<std::string>& sigma =
        (it_label != ts.label.end()) ? it_label->second
                                     : *new std::set<std::string>();
    for (const auto& act : ts.actions) {
        auto it = ts.trans.find(std::make_pair(s, act));
        if (it == ts.trans.end()) continue;
        for (const auto& s_next : it->second) {
            std::string q_next = ba.delta(q, sigma);
            out.push_back({ProdState{s_next, q_next}, act});
        }
    }
    return out;
}

int main() {
    TS ts;
    // Here you would fill ts.states, ts.initial, ts.trans, ts.label

    Buchi ba;
    ProdState start{ts.initial, ba.initial};

    std::queue<ProdState> q;
    std::set<ProdState,ProdStateLess> visited;
    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        ProdState cur = q.front(); q.pop();
        const std::string& q_ba = cur.second;
        bool is_accepting = ba.accepting.find(q_ba) != ba.accepting.end();
        if (is_accepting) {
            std::cout << "Found reachable accepting product state" << std::endl;
            break;
        }
        for (auto& succ : productSuccessors(ts, ba, cur)) {
            const ProdState& nxt = succ.first;
            if (visited.find(nxt) == visited.end()) {
                visited.insert(nxt);
                q.push(nxt);
            }
        }
    }
    return 0;
}
      
