// Chapter20_Lesson1.cpp
// Chapter 20 - Lesson 1
// Problem Definition and Environment Setup (C++ version)

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>

struct MissionSpec {
    std::string name;
    double start_x, start_y, start_theta;
    double goal_x, goal_y;
    double max_time_s;
    double min_success_prob;
    int max_collisions;
    double map_resolution_m;
};

struct SensorSuite {
    double lidar_range_m;
    double lidar_fov_deg;
    double lidar_rate_hz;
    double imu_rate_hz;
    double wheel_rate_hz;
    double gps_rate_hz;
    bool gps_available;
};

struct RobotLimits {
    double radius_m;
    double v_max_mps;
    double w_max_radps;
    double battery_wh;
    double avg_power_w;
};

struct EnvironmentConfig {
    double width_m;
    double height_m;
    double obstacle_density;
    int slip_regions;
    unsigned int seed;
};

struct Grid2D {
    int rows;
    int cols;
    std::vector<int> occ;          // 0 free, 1 occupied
    std::vector<double> trav_cost; // inf for occupied

    int idx(int r, int c) const { return r * cols + c; }
};

Grid2D createEnvironment(const EnvironmentConfig& env, double res) {
    int cols = static_cast<int>(std::ceil(env.width_m / res));
    int rows = static_cast<int>(std::ceil(env.height_m / res));
    Grid2D g{rows, cols, std::vector<int>(rows * cols, 0), std::vector<double>(rows * cols, 1.0)};

    std::mt19937 gen(env.seed);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            bool occ = (uni(gen) < env.obstacle_density);
            g.occ[g.idx(r, c)] = occ ? 1 : 0;
            g.trav_cost[g.idx(r, c)] = 1.0 + 0.5 * uni(gen);
        }
    }

    int margin = std::max(2, static_cast<int>(std::ceil(0.8 / res)));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (r < margin || r >= rows - margin || c < margin || c >= cols - margin) {
                g.occ[g.idx(r, c)] = 0;
            }
        }
    }

    // Slip regions (increase traversability cost in circular patches)
    std::uniform_int_distribution<int> rr(0, rows - 1);
    std::uniform_int_distribution<int> cc(0, cols - 1);
    int rad_min = std::max(3, std::min(rows, cols) / 20);
    int rad_max = std::max(4, std::min(rows, cols) / 8);
    std::uniform_int_distribution<int> rad_dist(rad_min, std::max(rad_min, rad_max));
    for (int k = 0; k < env.slip_regions; ++k) {
        int cy = rr(gen);
        int cx = cc(gen);
        int rad = rad_dist(gen);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                int dx = c - cx;
                int dy = r - cy;
                if (dx * dx + dy * dy <= rad * rad) {
                    g.trav_cost[g.idx(r, c)] += 1.0 + 0.8 * uni(gen);
                }
            }
        }
    }

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (g.occ[g.idx(r, c)] == 1) {
                g.trav_cost[g.idx(r, c)] = INFINITY;
            }
        }
    }

    return g;
}

bool pickStartGoal(const Grid2D& g, double res, unsigned int seed,
                   double& sx, double& sy, double& gx, double& gy) {
    std::vector<std::pair<int, int>> free_cells;
    for (int r = 0; r < g.rows; ++r) {
        for (int c = 0; c < g.cols; ++c) {
            if (g.occ[g.idx(r, c)] == 0) free_cells.push_back({r, c});
        }
    }
    if (free_cells.size() < 2) return false;

    std::mt19937 gen(seed + 20);
    std::uniform_int_distribution<size_t> pick(0, free_cells.size() - 1);

    for (int it = 0; it < 2000; ++it) {
        auto a = free_cells[pick(gen)];
        auto b = free_cells[pick(gen)];
        if (a == b) continue;
        double dx = (b.second - a.second) * res;
        double dy = (b.first - a.first) * res;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d >= 0.4 * std::max(g.rows, g.cols) * res) {
            sy = a.first * res; sx = a.second * res;
            gy = b.first * res; gx = b.second * res;
            return true;
        }
    }
    return false;
}

double batteryAvailableTime(const RobotLimits& robot) {
    return (robot.battery_wh / robot.avg_power_w) * 3600.0;
}

void writeScenarioJson(const std::string& path, const MissionSpec& mission,
                       const SensorSuite& sensors, const RobotLimits& robot,
                       const EnvironmentConfig& env, const Grid2D& g) {
    double occ_count = 0.0;
    double trav_sum = 0.0, trav_n = 0.0;
    for (int i = 0; i < static_cast<int>(g.occ.size()); ++i) {
        if (g.occ[i] == 1) occ_count += 1.0;
        if (std::isfinite(g.trav_cost[i])) {
            trav_sum += g.trav_cost[i];
            trav_n += 1.0;
        }
    }

    std::ofstream f(path);
    f << std::fixed << std::setprecision(4);
    f << "{\n";
    f << "  \"mission\": {\n";
    f << "    \"name\": \"" << mission.name << "\",\n";
    f << "    \"start_xytheta\": [" << mission.start_x << ", " << mission.start_y << ", " << mission.start_theta << "],\n";
    f << "    \"goal_xy\": [" << mission.goal_x << ", " << mission.goal_y << "],\n";
    f << "    \"max_time_s\": " << mission.max_time_s << ",\n";
    f << "    \"min_success_prob\": " << mission.min_success_prob << ",\n";
    f << "    \"max_collisions\": " << mission.max_collisions << ",\n";
    f << "    \"map_resolution_m\": " << mission.map_resolution_m << "\n";
    f << "  },\n";
    f << "  \"sensors\": {\n";
    f << "    \"lidar_range_m\": " << sensors.lidar_range_m << ",\n";
    f << "    \"lidar_fov_deg\": " << sensors.lidar_fov_deg << ",\n";
    f << "    \"lidar_rate_hz\": " << sensors.lidar_rate_hz << ",\n";
    f << "    \"imu_rate_hz\": " << sensors.imu_rate_hz << ",\n";
    f << "    \"wheel_rate_hz\": " << sensors.wheel_rate_hz << ",\n";
    f << "    \"gps_rate_hz\": " << sensors.gps_rate_hz << ",\n";
    f << "    \"gps_available\": " << (sensors.gps_available ? "true" : "false") << "\n";
    f << "  },\n";
    f << "  \"robot\": {\n";
    f << "    \"radius_m\": " << robot.radius_m << ",\n";
    f << "    \"v_max_mps\": " << robot.v_max_mps << ",\n";
    f << "    \"w_max_radps\": " << robot.w_max_radps << ",\n";
    f << "    \"battery_wh\": " << robot.battery_wh << ",\n";
    f << "    \"avg_power_w\": " << robot.avg_power_w << "\n";
    f << "  },\n";
    f << "  \"environment\": {\n";
    f << "    \"width_m\": " << env.width_m << ",\n";
    f << "    \"height_m\": " << env.height_m << ",\n";
    f << "    \"obstacle_density\": " << env.obstacle_density << ",\n";
    f << "    \"slip_regions\": " << env.slip_regions << ",\n";
    f << "    \"seed\": " << env.seed << "\n";
    f << "  },\n";
    f << "  \"occupancy_shape\": [" << g.rows << ", " << g.cols << "],\n";
    f << "  \"occupancy_occupied_fraction\": " << (occ_count / g.occ.size()) << ",\n";
    f << "  \"traversability_finite_mean\": " << (trav_n > 0 ? trav_sum / trav_n : 0.0) << "\n";
    f << "}\n";
}

int main() {
    EnvironmentConfig env{60.0, 40.0, 0.14, 4, 42};
    RobotLimits robot{0.25, 1.2, 1.8, 180.0, 85.0};
    SensorSuite sensors{15.0, 270.0, 10.0, 100.0, 50.0, 5.0, true};

    MissionSpec mission{
        "Capstone_AMR_IndoorOutdoor_Mix",
        0.0, 0.0, 0.0,
        1.0, 1.0,
        1200.0,
        0.90,
        0,
        0.2
    };

    Grid2D grid = createEnvironment(env, mission.map_resolution_m);

    double sx = 0.0, sy = 0.0, gx = 1.0, gy = 1.0;
    bool ok = pickStartGoal(grid, mission.map_resolution_m, env.seed, sx, sy, gx, gy);
    if (!ok) {
        std::cerr << "Failed to pick start/goal in free space\n";
        return 1;
    }

    mission.start_x = sx; mission.start_y = sy; mission.start_theta = 0.0;
    mission.goal_x = gx; mission.goal_y = gy;

    double t_available = batteryAvailableTime(robot);
    std::cout << "Mission: " << mission.name << "\n";
    std::cout << "Start (x,y,th): " << mission.start_x << ", " << mission.start_y << ", " << mission.start_theta << "\n";
    std::cout << "Goal (x,y): " << mission.goal_x << ", " << mission.goal_y << "\n";
    std::cout << "Battery available time [s]: " << t_available << "\n";
    std::cout << "Mission required time [s]: " << mission.max_time_s << "\n";
    std::cout << "Time feasible? " << (t_available >= mission.max_time_s ? "YES" : "NO") << "\n";

    bool rates_ok = (sensors.wheel_rate_hz >= 20.0) && (sensors.imu_rate_hz >= 50.0) && (sensors.lidar_rate_hz >= 5.0);
    std::cout << "Sensor rate checks ok? " << (rates_ok ? "YES" : "NO") << "\n";

    writeScenarioJson("Chapter20_Lesson1_cpp_scenario.json", mission, sensors, robot, env, grid);
    std::cout << "Wrote Chapter20_Lesson1_cpp_scenario.json\n";

    return 0;
}
