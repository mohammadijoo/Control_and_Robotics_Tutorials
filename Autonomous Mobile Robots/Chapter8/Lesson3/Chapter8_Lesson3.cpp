
/*
Chapter8_Lesson3.cpp

Autonomous Mobile Robots (Control Engineering) - Chapter 8, Lesson 3
Sensor Likelihoods for LiDAR and Vision

This file provides:
1) A 2D occupancy grid and approximate Euclidean distance transform (multi-source Dijkstra).
2) A LiDAR likelihood-field model.
3) A bearing-only vision landmark likelihood with outlier mixture.
4) Log-sum-exp normalization for particle weights.

Robotics ecosystem notes:
- ROS2 integration: subscribe to sensor_msgs::msg::LaserScan, build/update nav_msgs::msg::OccupancyGrid,
  and evaluate likelihoods per particle.
- Eigen can be used for vectorization; this example uses only the C++ standard library.

Build:
  g++ -O2 -std=c++17 Chapter8_Lesson3.cpp -o Chapter8_Lesson3
Run:
  ./Chapter8_Lesson3
*/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

static inline double wrapToPi(double a) {
    double twoPi = 2.0 * M_PI;
    a = std::fmod(a + M_PI, twoPi);
    if (a < 0) a += twoPi;
    return a - M_PI;
}

struct Pose2D {
    double x{0}, y{0}, theta{0};
};

struct OccupancyGrid2D {
    int w{0}, h{0};
    double resolution{0.05};
    double origin_x{-3.0}, origin_y{-3.0};
    std::vector<uint8_t> occ; // 1 obstacle, 0 free; row-major [y*w + x]

    OccupancyGrid2D(int w_, int h_, double res, double ox, double oy)
        : w(w_), h(h_), resolution(res), origin_x(ox), origin_y(oy), occ(static_cast<size_t>(w_*h_), 0) {}

    inline bool inBounds(int gx, int gy) const {
        return (0 <= gx && gx < w && 0 <= gy && gy < h);
    }

    inline int idx(int gx, int gy) const { return gy * w + gx; }

    void setObstacle(int gx, int gy) {
        if (inBounds(gx, gy)) occ[static_cast<size_t>(idx(gx, gy))] = 1;
    }

    inline uint8_t get(int gx, int gy) const { return occ[static_cast<size_t>(idx(gx, gy))]; }

    std::pair<int,int> worldToGrid(double x, double y) const {
        int gx = static_cast<int>(std::floor((x - origin_x) / resolution));
        int gy = static_cast<int>(std::floor((y - origin_y) / resolution));
        return {gx, gy};
    }
};

std::vector<double> distanceTransformBrushfire(const OccupancyGrid2D& grid) {
    // Multi-source Dijkstra on 8-neighborhood, step costs 1 and sqrt(2), then scale by resolution.
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(static_cast<size_t>(grid.w * grid.h), INF);

    using Node = std::tuple<double,int,int>; // (d, gx, gy)
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            return std::get<0>(a) > std::get<0>(b); // min-heap by distance
        }
    };
    std::priority_queue<Node, std::vector<Node>, Cmp> pq;

    for (int gy = 0; gy < grid.h; ++gy) {
        for (int gx = 0; gx < grid.w; ++gx) {
            if (grid.get(gx, gy) == 1) {
                int id = grid.idx(gx, gy);
                dist[static_cast<size_t>(id)] = 0.0;
                pq.emplace(0.0, gx, gy);
            }
        }
    }

    const int nbh[8][2] = { {-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1} };

    while (!pq.empty()) {
        auto [d, gx, gy] = pq.top();
        pq.pop();
        int id = grid.idx(gx, gy);
        if (d > dist[static_cast<size_t>(id)]) continue;

        for (auto &off : nbh) {
            int ngx = gx + off[0], ngy = gy + off[1];
            if (!grid.inBounds(ngx, ngy)) continue;
            double step = (off[0] == 0 || off[1] == 0) ? 1.0 : std::sqrt(2.0);
            double nd = d + step;
            int nid = grid.idx(ngx, ngy);
            if (nd < dist[static_cast<size_t>(nid)]) {
                dist[static_cast<size_t>(nid)] = nd;
                pq.emplace(nd, ngx, ngy);
            }
        }
    }

    for (auto &v : dist) v *= grid.resolution;
    return dist;
}

struct LikelihoodFieldModel {
    const OccupancyGrid2D& grid;
    const std::vector<double>& dist_m;
    double z_max{8.0};
    double sigma_hit{0.15};
    double w_hit{0.95};
    double w_rand{0.05};
    double max_dist{2.0};

    double endpointDistance(double wx, double wy) const {
        auto [gx, gy] = grid.worldToGrid(wx, wy);
        if (!grid.inBounds(gx, gy)) return max_dist;
        double d = dist_m[static_cast<size_t>(grid.idx(gx, gy))];
        return std::min(max_dist, d);
    }

    double logLikelihood(const Pose2D& pose,
                         const std::vector<double>& ranges,
                         const std::vector<double>& rel_angles) const {
        const double sig2 = sigma_hit * sigma_hit;
        const double invz = 1.0 / z_max;
        const size_t n = std::min(ranges.size(), rel_angles.size());
        double logp = 0.0;

        for (size_t i = 0; i < n; ++i) {
            double z = std::clamp(ranges[i], 0.0, z_max);
            double a = pose.theta + rel_angles[i];
            double wx = pose.x + z * std::cos(a);
            double wy = pose.y + z * std::sin(a);

            double d = endpointDistance(wx, wy);
            double p_hit = std::exp(-(d*d) / (2.0*sig2));
            double p = w_hit * p_hit + w_rand * invz;
            logp += std::log(std::max(p, 1e-12));
        }
        return logp;
    }
};

struct VisionBearingModel {
    std::unordered_map<int, std::pair<double,double>> landmarks; // id -> (x,y)
    double sigma_bearing{(3.0 * M_PI / 180.0)};
    double eps_outlier{0.10};

    double logLikelihood(const Pose2D& pose,
                         const std::vector<int>& ids,
                         const std::vector<double>& bearings) const {
        const double sig2 = sigma_bearing * sigma_bearing;
        const double norm = 1.0 / std::sqrt(2.0 * M_PI * sig2);
        const double uni  = 1.0 / (2.0 * M_PI);
        const size_t n = std::min(ids.size(), bearings.size());

        double logp = 0.0;
        for (size_t i = 0; i < n; ++i) {
            auto it = landmarks.find(ids[i]);
            if (it == landmarks.end()) {
                logp += std::log(uni);
                continue;
            }
            const auto [lx, ly] = it->second;
            double pred = wrapToPi(std::atan2(ly - pose.y, lx - pose.x) - pose.theta);
            double innov = wrapToPi(bearings[i] - pred);

            double p_in = norm * std::exp(-(innov*innov)/(2.0*sig2));
            double p = (1.0 - eps_outlier) * p_in + eps_outlier * uni;
            logp += std::log(std::max(p, 1e-15));
        }
        return logp;
    }
};

static inline double logSumExp(const std::vector<double>& logw) {
    double m = *std::max_element(logw.begin(), logw.end());
    double s = 0.0;
    for (double v : logw) s += std::exp(v - m);
    return m + std::log(s);
}

static inline std::vector<double> normalizeLogWeights(const std::vector<double>& logw) {
    double lse = logSumExp(logw);
    std::vector<double> w(logw.size());
    double sum = 0.0;
    for (size_t i = 0; i < logw.size(); ++i) {
        w[i] = std::exp(logw[i] - lse);
        sum += w[i];
    }
    for (auto &v : w) v /= sum;
    return w;
}

int main() {
    // Create a synthetic map: wall + pillar
    OccupancyGrid2D grid(120, 120, 0.05, -3.0, -3.0);
    for (int gx = 10; gx < 110; ++gx) grid.setObstacle(gx, 60);
    for (int gy = 20; gy < 30; ++gy)
        for (int gx = 85; gx < 95; ++gx)
            grid.setObstacle(gx, gy);

    auto dist = distanceTransformBrushfire(grid);

    LikelihoodFieldModel lidar{grid, dist, 8.0, 0.15, 0.95, 0.05, 2.0};

    VisionBearingModel vision;
    vision.landmarks = {
        {1, {-1.0,  0.0} },
        {2, { 1.0,  0.5} },
        {3, { 0.0, -1.0} }
    };
    vision.sigma_bearing = 3.0 * M_PI / 180.0;
    vision.eps_outlier = 0.10;

    std::vector<Pose2D> particles = {
        {-0.5, -0.8, 90.0 * M_PI / 180.0},
        {-0.2, -0.6, 88.0 * M_PI / 180.0},
        { 0.8,  0.1, 30.0 * M_PI / 180.0}
    };

    // Synthetic scan: 9 beams
    std::vector<double> rel_angles;
    rel_angles.reserve(9);
    for (int i = 0; i < 9; ++i) rel_angles.push_back((-40.0 + 80.0 * i / 8.0) * M_PI / 180.0);
    std::vector<double> ranges = {2.8, 3.0, 3.2, 3.1, 3.0, 3.2, 2.9, 2.7, 2.6};

    // Vision observations
    std::vector<int> ids = {1,2,3};
    std::vector<double> bears = {70.0 * M_PI / 180.0, 20.0 * M_PI / 180.0, -95.0 * M_PI / 180.0};

    std::vector<double> logw;
    logw.reserve(particles.size());
    for (const auto& p : particles) {
        double ll_l = lidar.logLikelihood(p, ranges, rel_angles);
        double ll_v = vision.logLikelihood(p, ids, bears);
        logw.push_back(ll_l + ll_v);
    }

    auto w = normalizeLogWeights(logw);

    std::cout << "log-weights: ";
    for (double v : logw) std::cout << v << " ";
    std::cout << "\nweights:     ";
    for (double v : w) std::cout << v << " ";
    std::cout << "\n";
    return 0;
}
      