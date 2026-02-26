// Chapter19_Lesson2.cpp
/*
Metrics for Navigation Robustness (Autonomous Mobile Robots)

Build:
  g++ -std=c++17 -O2 -o Chapter19_Lesson2 Chapter19_Lesson2.cpp

Usage:
  ./Chapter19_Lesson2            # runs on embedded synthetic episodes
  ./Chapter19_Lesson2 log.csv    # simple CSV parser (schema documented below)

CSV schema (header required):
episode_id,time_s,x,y,goal_x,goal_y,clearance_m,collision,intervention,goal_reached,
tracking_error_m,cmd_v,cmd_v_max,cmd_w,cmd_w_max,recovery_event

Robotics library integration notes:
- For ROS 2 / Nav2, export odom (x,y), goal pose, cmd_vel, local minimum clearance,
  and behavior-tree events (collision, intervention, recovery) into a CSV logger.
- Suggested libraries in production: rclcpp, nav_msgs, geometry_msgs, Eigen.
*/

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

struct Sample {
    int episode_id{};
    double time_s{};
    double x{}, y{};
    double goal_x{}, goal_y{};
    double clearance_m{};
    int collision{};
    int intervention{};
    int goal_reached{};
    double tracking_error_m{};
    double cmd_v{}, cmd_v_max{};
    double cmd_w{}, cmd_w_max{};
    int recovery_event{};
};

struct EpisodeMetrics {
    int episode_id{};
    int success{};
    int collision_free{};
    int intervention_free{};
    int had_failure{};
    int recovered_after_failure{};
    double completion_time_s{};
    double path_length_m{};
    double reference_distance_m{};
    double path_efficiency{};
    double min_clearance_m{};
    double tracking_rmse_m{};
    double saturation_ratio{};
};

struct Wilson {
    double p_hat{};
    double lo{};
    double hi{};
};

static Wilson wilson_interval(int k, int n, double z = 1.96) {
    if (n <= 0) return {NAN, NAN, NAN};
    double p = static_cast<double>(k) / n;
    double denom = 1.0 + (z * z) / n;
    double center = (p + (z * z) / (2.0 * n)) / denom;
    double rad = (z / denom) * std::sqrt((p * (1.0 - p) / n) + (z * z) / (4.0 * n * n));
    return {p, center - rad, center + rad};
}

static double dist2d(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

static std::vector<Sample> make_synthetic_data() {
    // deterministic pseudo-data for reproducibility (no RNG dependency)
    std::vector<Sample> out;
    for (int ep = 0; ep < 8; ++ep) {
        double x = 0.0, y = 0.2 * (ep % 2);
        double gx = 8.0 + 0.3 * ep;
        double gy = (ep % 3) - 1.0;
        bool terminated = false;
        for (int k = 0; k < 160 && !terminated; ++k) {
            double t = 0.1 * k;
            double dx = gx - x;
            double dy = gy - y;
            double d = std::sqrt(dx * dx + dy * dy);
            double theta = std::atan2(dy, dx);
            double cmd_v_max = 0.8, cmd_w_max = 1.2;
            double cmd_v = std::min(cmd_v_max, 0.55 + 0.05 * std::sin(0.07 * k + ep));
            double cmd_w = std::max(-cmd_w_max, std::min(cmd_w_max, 0.35 * std::sin(0.05 * k)));
            x += 0.1 * cmd_v * std::cos(theta);
            y += 0.1 * cmd_v * std::sin(theta);

            double clearance = 0.65 + 0.22 * std::sin(0.11 * k + 0.4 * ep);
            if (ep == 2 && k > 60 && k < 75) clearance -= 0.55;
            if (ep == 5 && k > 90 && k < 102) clearance -= 0.48;
            if (clearance < 0.0) clearance = 0.0;

            int collision = (clearance < 0.05) ? 1 : 0;
            int intervention = (clearance < 0.12 && !collision) ? 1 : 0;
            int goal_reached = (d < 0.30 && collision == 0) ? 1 : 0;
            int recovery = ((intervention == 1) && (k % 5 == 0)) ? 1 : 0;
            double e_track = std::fabs(0.05 * std::sin(0.02 * k + ep));

            out.push_back({ep, t, x, y, gx, gy, clearance, collision, intervention, goal_reached,
                           e_track, cmd_v, cmd_v_max, cmd_w, cmd_w_max, recovery});

            if (collision || goal_reached) terminated = true;
        }
    }
    return out;
}

static std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) cols.push_back(item);
    return cols;
}

static bool read_csv(const std::string& path, std::vector<Sample>& out) {
    std::ifstream fin(path);
    if (!fin) return false;
    std::string line;
    if (!std::getline(fin, line)) return false; // header
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        auto c = split_csv_line(line);
        if (c.size() < 16) continue;
        Sample s;
        s.episode_id = std::stoi(c[0]);
        s.time_s = std::stod(c[1]);
        s.x = std::stod(c[2]); s.y = std::stod(c[3]);
        s.goal_x = std::stod(c[4]); s.goal_y = std::stod(c[5]);
        s.clearance_m = std::stod(c[6]);
        s.collision = std::stoi(c[7]);
        s.intervention = std::stoi(c[8]);
        s.goal_reached = std::stoi(c[9]);
        s.tracking_error_m = std::stod(c[10]);
        s.cmd_v = std::stod(c[11]); s.cmd_v_max = std::stod(c[12]);
        s.cmd_w = std::stod(c[13]); s.cmd_w_max = std::stod(c[14]);
        s.recovery_event = std::stoi(c[15]);
        out.push_back(s);
    }
    return !out.empty();
}

int main(int argc, char** argv) {
    std::vector<Sample> data;
    if (argc >= 2) {
        if (!read_csv(argv[1], data)) {
            std::cerr << "Failed to read CSV. Falling back to synthetic data.\n";
            data = make_synthetic_data();
        }
    } else {
        data = make_synthetic_data();
    }

    std::map<int, std::vector<Sample>> by_ep;
    for (const auto& s : data) by_ep[s.episode_id].push_back(s);

    std::vector<EpisodeMetrics> eps;
    for (auto& kv : by_ep) {
        auto& g = kv.second;
        std::sort(g.begin(), g.end(), [](const Sample& a, const Sample& b){ return a.time_s < b.time_s; });

        int success = 0, collisions = 0, interventions = 0, rec = 0;
        double min_clear = 1e9;
        double sum_e2 = 0.0;
        int sat_count = 0;
        double path_len = 0.0;
        for (size_t i = 0; i < g.size(); ++i) {
            success = std::max(success, g[i].goal_reached);
            collisions += g[i].collision;
            interventions += g[i].intervention;
            rec += g[i].recovery_event;
            min_clear = std::min(min_clear, g[i].clearance_m);
            sum_e2 += g[i].tracking_error_m * g[i].tracking_error_m;
            bool sat_v = std::fabs(g[i].cmd_v) >= 0.98 * std::max(g[i].cmd_v_max, 1e-9);
            bool sat_w = std::fabs(g[i].cmd_w) >= 0.98 * std::max(g[i].cmd_w_max, 1e-9);
            if (sat_v || sat_w) sat_count++;
            if (i > 0) path_len += dist2d(g[i-1].x, g[i-1].y, g[i].x, g[i].y);
        }
        double t_final = g.back().time_s - g.front().time_s;
        double d_ref = dist2d(g.front().x, g.front().y, g.front().goal_x, g.front().goal_y);
        double eta = success ? std::min(1.0, d_ref / std::max(path_len, 1e-9)) : 0.0;
        int had_failure = (collisions + interventions) > 0 ? 1 : 0;
        int recovered_after_failure = (had_failure && rec > 0 && success) ? 1 : 0;

        eps.push_back({
            kv.first,
            success,
            collisions == 0 ? 1 : 0,
            interventions == 0 ? 1 : 0,
            had_failure,
            recovered_after_failure,
            t_final,
            path_len,
            d_ref,
            eta,
            min_clear,
            std::sqrt(sum_e2 / std::max<size_t>(1, g.size())),
            static_cast<double>(sat_count) / std::max<size_t>(1, g.size())
        });
    }

    int n = static_cast<int>(eps.size());
    int k_success = 0, k_collision_free = 0, k_intervention_free = 0, k_recovered = 0, n_failed = 0;
    double mean_eta = 0.0, mean_clear = 0.0, mean_track = 0.0, mean_t = 0.0;
    int n_success = 0;
    for (const auto& e : eps) {
        k_success += e.success;
        k_collision_free += e.collision_free;
        k_intervention_free += e.intervention_free;
        k_recovered += e.recovered_after_failure;
        n_failed += e.had_failure;
        mean_eta += e.path_efficiency;
        mean_clear += e.min_clearance_m;
        mean_track += e.tracking_rmse_m;
        if (e.success) {
            mean_t += e.completion_time_s;
            n_success++;
        }
    }
    mean_eta /= std::max(1, n);
    mean_clear /= std::max(1, n);
    mean_track /= std::max(1, n);
    mean_t = (n_success > 0) ? mean_t / n_success : NAN;

    Wilson w_succ = wilson_interval(k_success, n);
    Wilson w_safe = wilson_interval(k_collision_free, n);

    auto exp_score = [](double x, double tau){ return std::exp(-x / tau); };
    double time_score = 0.0, track_score = 0.0, clear_score = 0.0;
    for (const auto& e : eps) {
        time_score += exp_score(e.completion_time_s, 60.0) * e.success;
        track_score += exp_score(e.tracking_rmse_m, 0.25);
        clear_score += std::min(1.0, std::max(0.0, e.min_clearance_m / 0.30));
    }
    time_score /= std::max(1, n);
    track_score /= std::max(1, n);
    clear_score /= std::max(1, n);

    double success_rate = static_cast<double>(k_success) / std::max(1, n);
    double collision_free_rate = static_cast<double>(k_collision_free) / std::max(1, n);
    double intervention_free_rate = static_cast<double>(k_intervention_free) / std::max(1, n);
    double robustness =
        0.30 * collision_free_rate +
        0.15 * intervention_free_rate +
        0.20 * success_rate +
        0.15 * clear_score +
        0.10 * mean_eta +
        0.05 * track_score +
        0.05 * time_score;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Per-episode metrics:\n";
    for (const auto& e : eps) {
        std::cout << "ep=" << e.episode_id
                  << " success=" << e.success
                  << " collision_free=" << e.collision_free
                  << " intervention_free=" << e.intervention_free
                  << " T=" << e.completion_time_s
                  << " L=" << e.path_length_m
                  << " eta=" << e.path_efficiency
                  << " min_clear=" << e.min_clearance_m
                  << " track_rmse=" << e.tracking_rmse_m
                  << " sat=" << e.saturation_ratio
                  << "\n";
    }

    std::cout << "\nSummary metrics:\n";
    std::cout << "N episodes                    = " << n << "\n";
    std::cout << "Success rate                 = " << success_rate
              << " (Wilson 95%: " << w_succ.lo << ", " << w_succ.hi << ")\n";
    std::cout << "Collision-free rate          = " << collision_free_rate
              << " (Wilson 95%: " << w_safe.lo << ", " << w_safe.hi << ")\n";
    std::cout << "Intervention-free rate       = " << intervention_free_rate << "\n";
    std::cout << "Mean completion time (succ)  = " << mean_t << " s\n";
    std::cout << "Mean path efficiency         = " << mean_eta << "\n";
    std::cout << "Mean min clearance           = " << mean_clear << " m\n";
    std::cout << "Mean tracking RMSE           = " << mean_track << " m\n";
    std::cout << "Recovery-after-failure rate  = "
              << ((n_failed > 0) ? (static_cast<double>(k_recovered) / n_failed) : NAN) << "\n";
    std::cout << "Composite robustness score   = " << robustness << "\n";

    return 0;
}
