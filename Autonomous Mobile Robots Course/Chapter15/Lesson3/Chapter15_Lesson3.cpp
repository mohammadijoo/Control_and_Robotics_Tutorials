/*
Chapter15_Lesson3.cpp
Dynamic Window Approach (DWA) — minimal C++17 implementation (no ROS required).
Build (example):
  g++ -std=c++17 -O2 Chapter15_Lesson3.cpp -o dwa_demo
*/

#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

struct State {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double v{0.0};
    double w{0.0};
};

struct Config {
    // Kinematic limits
    double v_min{-0.2};
    double v_max{ 1.0};
    double w_min{-1.5};
    double w_max{ 1.5};

    // Dynamics
    double a_max{0.8};
    double a_brake{1.0};
    double alpha_max{2.0};

    // Discretization
    double dt{0.1};
    double T{2.0};
    double v_res{0.05};
    double w_res{0.1};

    // Geometry
    double robot_radius{0.3};

    // Weights
    double w_heading{0.4};
    double w_clear{0.4};
    double w_speed{0.2};
};

static double wrap_to_pi(double a) {
    const double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2.0 * pi);
    if (a < 0) a += 2.0 * pi;
    return a - pi;
}

static State motion_step(const State& s, double v, double w, double dt) {
    State ns = s;
    ns.x += v * std::cos(s.theta) * dt;
    ns.y += v * std::sin(s.theta) * dt;
    ns.theta += w * dt;
    ns.v = v;
    ns.w = w;
    return ns;
}

static std::tuple<double,double,double,double> dynamic_window(const State& s, const Config& cfg) {
    double v_low_dyn = s.v - cfg.a_brake * cfg.dt;
    double v_high_dyn = s.v + cfg.a_max * cfg.dt;
    double w_low_dyn = s.w - cfg.alpha_max * cfg.dt;
    double w_high_dyn = s.w + cfg.alpha_max * cfg.dt;

    double vL = std::max(cfg.v_min, v_low_dyn);
    double vU = std::min(cfg.v_max, v_high_dyn);
    double wL = std::max(cfg.w_min, w_low_dyn);
    double wU = std::min(cfg.w_max, w_high_dyn);
    return {vL, vU, wL, wU};
}

static std::vector<State> rollout(const State& s0, double v, double w, const Config& cfg) {
    int n = static_cast<int>(cfg.T / cfg.dt);
    std::vector<State> traj;
    traj.reserve(n + 1);
    traj.push_back(s0);
    State s = s0;
    for (int i = 0; i < n; ++i) {
        s = motion_step(s, v, w, cfg.dt);
        traj.push_back(s);
    }
    return traj;
}

static double min_clearance(const std::vector<State>& traj, const std::vector<std::pair<double,double>>& obs) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& st : traj) {
        for (const auto& o : obs) {
            double dx = st.x - o.first;
            double dy = st.y - o.second;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < best) best = d;
        }
    }
    return best;
}

static double heading_score(const std::vector<State>& traj, const std::pair<double,double>& goal) {
    const auto& last = traj.back();
    double dir = std::atan2(goal.second - last.y, goal.first - last.x);
    double err = wrap_to_pi(dir - last.theta);
    return std::cos(err); // [-1,1]
}

static double speed_score(double v, const Config& cfg) {
    return (v - cfg.v_min) / std::max(1e-9, (cfg.v_max - cfg.v_min));
}

static bool admissible(double v, double clearance, const Config& cfg) {
    double d_stop = (std::max(0.0, v) * std::max(0.0, v)) / std::max(1e-9, 2.0 * cfg.a_brake);
    return clearance > (d_stop + cfg.robot_radius);
}

static void normalize(std::vector<double>& vals) {
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (double v : vals) { lo = std::min(lo, v); hi = std::max(hi, v); }
    if (std::fabs(hi - lo) < 1e-12) {
        for (auto& v : vals) v = 0.0;
        return;
    }
    for (auto& v : vals) v = (v - lo) / (hi - lo);
}

struct Candidate {
    double h;
    double c;
    double s;
    double v;
    double w;
    std::vector<State> traj;
};

static std::tuple<double,double,std::vector<State>>
dwa_control(const State& s, const std::pair<double,double>& goal,
            const std::vector<std::pair<double,double>>& obs, const Config& cfg) {

    auto [vL, vU, wL, wU] = dynamic_window(s, cfg);

    std::vector<Candidate> cand;
    for (double v = vL; v <= vU + 1e-12; v += cfg.v_res) {
        for (double w = wL; w <= wU + 1e-12; w += cfg.w_res) {
            auto traj = rollout(s, v, w, cfg);
            double clear = min_clearance(traj, obs) - cfg.robot_radius;
            if (!admissible(v, clear, cfg)) continue;
            Candidate c;
            c.h = heading_score(traj, goal);
            c.c = clear;
            c.s = speed_score(v, cfg);
            c.v = v;
            c.w = w;
            c.traj = std::move(traj);
            cand.push_back(std::move(c));
        }
    }

    if (cand.empty()) {
        return {0.0, 0.0, std::vector<State>{s}};
    }

    std::vector<double> hs, cs, ss;
    hs.reserve(cand.size()); cs.reserve(cand.size()); ss.reserve(cand.size());
    for (const auto& c : cand) { hs.push_back(c.h); cs.push_back(c.c); ss.push_back(c.s); }
    normalize(hs); normalize(cs); normalize(ss);

    double bestJ = -1e18;
    size_t bestIdx = 0;
    for (size_t i = 0; i < cand.size(); ++i) {
        double J = cfg.w_heading * hs[i] + cfg.w_clear * cs[i] + cfg.w_speed * ss[i];
        if (J > bestJ) { bestJ = J; bestIdx = i; }
    }

    return {cand[bestIdx].v, cand[bestIdx].w, cand[bestIdx].traj};
}

int main() {
    Config cfg;
    State s;
    std::pair<double,double> goal{8.0, 6.0};

    std::vector<std::pair<double,double>> obs;
    for (int i = 0; i < 15; ++i) obs.push_back({2.0 + i*(5.0/14.0), 3.0});
    for (int i = 0; i < 12; ++i) obs.push_back({4.0, 1.0 + i*(4.0/11.0)});

    for (int k = 0; k < 600; ++k) {
        auto [v, w, traj] = dwa_control(s, goal, obs, cfg);
        s = motion_step(s, v, w, cfg.dt);
        double dist = std::hypot(s.x - goal.first, s.y - goal.second);
        if (k % 20 == 0) {
            std::cout << "k=" << k << " x=" << s.x << " y=" << s.y << " dist=" << dist << " v=" << v << " w=" << w << "\n";
        }
        if (dist < 0.3) break;
    }
    std::cout << "Final: x=" << s.x << " y=" << s.y << " theta=" << s.theta << "\n";
    return 0;
}
