// Chapter16_Lesson2.cpp
// Autonomous Mobile Robots — Chapter 16, Lesson 2: Velocity Obstacles (VO) Sampling Demo
//
// Educational reference implementation:
// - Time-horizon collision check in relative motion (quadratic)
// - Sampling-based velocity choice outside VO
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter16_Lesson2.cpp -o vo_demo
//
// Note: This is a minimal educational demo, not a full ORCA solver.

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

struct Vec2 {
    double x{0.0}, y{0.0};
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return Vec2{x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return Vec2{x-o.x, y-o.y}; }
    Vec2 operator*(double s) const { return Vec2{x*s, y*s}; }
};

static inline double dot(const Vec2& a, const Vec2& b) { return a.x*b.x + a.y*b.y; }
static inline double norm2(const Vec2& a) { return dot(a,a); }
static inline double norm(const Vec2& a) { return std::sqrt(norm2(a)); }

static inline Vec2 clampNorm(const Vec2& v, double vmax) {
    double n = norm(v);
    if (n <= vmax) return v;
    double s = vmax / (n + 1e-12);
    return v * s;
}

struct Agent {
    Vec2 p;
    Vec2 v;
    Vec2 goal;
    double radius{0.35};
    double vmax{1.2};
};

struct TTCResult {
    bool collides;
    double t_hit;
};

TTCResult timeToCollisionInHorizon(const Vec2& p_rel, const Vec2& v_rel, double R, double T) {
    if (norm(p_rel) <= R) return {true, 0.0};

    double a = norm2(v_rel);
    double b = 2.0 * dot(p_rel, v_rel);
    double c = norm2(p_rel) - R*R;

    if (a < 1e-12) return {false, std::numeric_limits<double>::infinity()};

    double disc = b*b - 4.0*a*c;
    if (disc < 0.0) return {false, std::numeric_limits<double>::infinity()};

    double sdisc = std::sqrt(disc);
    double t1 = (-b - sdisc) / (2.0*a);
    double t2 = (-b + sdisc) / (2.0*a);

    double t_hit = std::numeric_limits<double>::infinity();
    if (t1 >= 0.0) t_hit = t1;
    else if (t2 >= 0.0) t_hit = 0.0;

    if (t_hit >= 0.0 && t_hit <= T) return {true, t_hit};
    return {false, std::numeric_limits<double>::infinity()};
}

std::vector<Vec2> sampleDisk(double vmax, int n, std::mt19937& rng) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Vec2> out;
    out.reserve(n);
    for (int i=0;i<n;i++) {
        double ang = 2.0*M_PI*uni(rng);
        double r = std::sqrt(uni(rng))*vmax;
        out.emplace_back(r*std::cos(ang), r*std::sin(ang));
    }
    return out;
}

Vec2 chooseVelocityVO(const Agent& a, const std::vector<Agent>& neighbors, double dt, double T, std::mt19937& rng) {
    Vec2 toGoal = a.goal - a.p;
    double d = norm(toGoal);
    Vec2 v_pref{0.0, 0.0};
    if (d > 1e-9) {
        double spd = std::min(a.vmax, d / std::max(dt, 1e-3));
        v_pref = toGoal * (spd / (d + 1e-12));
    }

    auto cand = sampleDisk(a.vmax, 1200, rng);
    cand.push_back(clampNorm(v_pref, a.vmax));

    Vec2 best{0.0, 0.0};
    double bestCost = std::numeric_limits<double>::infinity();

    for (const auto& v : cand) {
        bool feasible = true;
        double min_ttc = std::numeric_limits<double>::infinity();

        for (const auto& nb : neighbors) {
            Vec2 p_rel = nb.p - a.p;
            Vec2 v_rel = v - nb.v;
            double R = a.radius + nb.radius;

            auto res = timeToCollisionInHorizon(p_rel, v_rel, R, T);
            if (res.collides) { feasible = false; break; }

            auto resAny = timeToCollisionInHorizon(p_rel, v_rel, R, 1e6);
            if (resAny.collides) min_ttc = std::min(min_ttc, resAny.t_hit);
        }

        if (!feasible) continue;

        double prefCost = norm2(v - v_pref);
        double safetyCost = 0.0;
        if (std::isfinite(min_ttc)) safetyCost = 1.0 / (min_ttc + 1e-6);

        double cost = 1.0*prefCost + 2.0*safetyCost;
        if (cost < bestCost) { bestCost = cost; best = v; }
    }

    if (!std::isfinite(bestCost)) return a.v * 0.2; // failsafe slow down
    return best;
}

int main() {
    std::mt19937 rng(1);

    const int N = 8;
    const int steps = 400;
    const double dt = 0.05;
    const double T = 2.5;
    const double R0 = 5.0;

    std::vector<Agent> agents;
    agents.reserve(N);

    for (int i=0;i<N;i++) {
        double ang = 2.0*M_PI*i/N;
        Vec2 p{R0*std::cos(ang), R0*std::sin(ang)};
        Agent a;
        a.p = p;
        a.v = Vec2{0.0, 0.0};
        a.goal = Vec2{-p.x, -p.y};
        agents.push_back(a);
    }

    for (int k=0;k<steps;k++) {
        std::vector<Vec2> nextV(N);
        for (int i=0;i<N;i++) {
            std::vector<Agent> neigh;
            neigh.reserve(N-1);
            for (int j=0;j<N;j++) if (j!=i) neigh.push_back(agents[j]);
            nextV[i] = chooseVelocityVO(agents[i], neigh, dt, T, rng);
        }
        for (int i=0;i<N;i++) {
            agents[i].v = nextV[i];
            agents[i].p = agents[i].p + agents[i].v * dt;
        }
        if (k % 50 == 0) {
            std::cout << "Step " << k << ": agent0 p=("
                      << agents[0].p.x << "," << agents[0].p.y << ")\n";
        }
    }

    std::cout << "Done.\n";
    return 0;
}
