#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <cmath>

using Fluent = std::string;
using State  = std::set<Fluent>;

struct Action {
    std::string name;
    State pre;
    State add;
    State del;

    bool applicable(const State& s) const {
        for (const auto& f : pre) {
            if (!s.count(f)) return false;
        }
        return true;
    }

    State apply(const State& s) const {
        State out = s;
        for (const auto& f : del) out.erase(f);
        for (const auto& f : add) out.insert(f);
        return out;
    }
};

struct Config2D {
    double x{0.0}, y{0.0};
};

bool inCollision(const Config2D& q) {
    const Config2D c{0.5, 0.5};
    const double r = 0.2;
    const double dx = q.x - c.x;
    const double dy = q.y - c.y;
    return std::sqrt(dx * dx + dy * dy) <= r;
}

bool straightLineFree(const Config2D& q0, const Config2D& q1, int steps = 50) {
    for (int i = 0; i <= steps; ++i) {
        double alpha = static_cast<double>(i) / steps;
        Config2D q{(1 - alpha) * q0.x + alpha * q1.x,
                   (1 - alpha) * q0.y + alpha * q1.y};
        if (inCollision(q)) return false;
    }
    return true;
}

// Refinement table
struct StateHash {
    std::size_t operator()(State const& s) const noexcept {
        std::size_t h = 0;
        for (const auto& f : s) {
            h ^= std::hash<std::string>{}(f) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        }
        return h;
    }
};

struct StateEq {
    bool operator()(State const& a, State const& b) const noexcept {
        return a == b;
    }
};

std::unordered_map<State, Config2D, StateHash, StateEq> Rtable;

std::optional<Config2D> refine(const State& s) {
    for (const auto& kv : Rtable) {
        const State& key = kv.first;
        bool subset = true;
        for (const auto& f : key) {
            if (!s.count(f)) { subset = false; break; }
        }
        if (subset) return kv.second;
    }
    return std::nullopt;
}

bool geomFeasible(const State& s, const Action& a, const State& sNext) {
    auto q  = refine(s);
    auto qn = refine(sNext);
    if (!q || !qn) return false;
    if (inCollision(*q) || inCollision(*qn)) return false;
    return straightLineFree(*q, *qn);
}
      
