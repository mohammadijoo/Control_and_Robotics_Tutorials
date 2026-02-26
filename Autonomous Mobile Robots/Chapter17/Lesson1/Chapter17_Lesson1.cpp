/*
Autonomous Mobile Robots — Chapter 17, Lesson 1: Frontier-Based Exploration
File: Chapter17_Lesson1.cpp

Minimal, self-contained frontier detection + clustering + scoring on a grid.
Grid encoding:
  -1 = unknown, 0 = free, 100 = occupied

This implementation is "library-free" for teaching. In a robot stack, you would:
  - ingest nav_msgs::OccupancyGrid (ROS2)
  - compute frontiers
  - select goal, send to Nav2 (NavigateToPose action)
*/

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>
#include <set>

using Coord = std::pair<int,int>; // (row, col)

static inline bool in_bounds(int H, int W, int r, int c) {
  return (0 <= r && r < H && 0 <= c && c < W);
}

static inline bool is_free(int v) { return v == 0; }
static inline bool is_unknown(int v) { return v == -1; }
static inline bool is_occupied(int v) { return v >= 50; }

static std::vector<Coord> neighbors4(int H, int W, Coord u) {
  const int r = u.first, c = u.second;
  std::vector<Coord> out;
  const int dr[4] = {-1, 1, 0, 0};
  const int dc[4] = {0, 0, -1, 1};
  for (int k = 0; k < 4; ++k) {
    int rr = r + dr[k], cc = c + dc[k];
    if (in_bounds(H, W, rr, cc)) out.push_back({rr, cc});
  }
  return out;
}

static std::vector<Coord> neighbors8(int H, int W, Coord u) {
  const int r = u.first, c = u.second;
  std::vector<Coord> out;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) continue;
      int rr = r + dr, cc = c + dc;
      if (in_bounds(H, W, rr, cc)) out.push_back({rr, cc});
    }
  }
  return out;
}

static std::vector<Coord> detect_frontiers(const std::vector<std::vector<int>>& grid) {
  const int H = (int)grid.size();
  const int W = (int)grid[0].size();
  std::vector<Coord> frontiers;
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      if (!is_free(grid[r][c])) continue;
      for (auto v : neighbors4(H, W, {r,c})) {
        if (is_unknown(grid[v.first][v.second])) { frontiers.push_back({r,c}); break; }
      }
    }
  }
  return frontiers;
}

static std::vector<std::vector<Coord>> cluster_frontiers(const std::vector<std::vector<int>>& grid,
                                                         const std::vector<Coord>& frontier_cells) {
  const int H = (int)grid.size();
  const int W = (int)grid[0].size();

  std::set<Coord> frontier_set(frontier_cells.begin(), frontier_cells.end());
  std::set<Coord> visited;
  std::vector<std::vector<Coord>> clusters;

  for (auto cell : frontier_cells) {
    if (visited.count(cell)) continue;
    if (!frontier_set.count(cell)) continue;

    std::queue<Coord> q;
    q.push(cell);
    visited.insert(cell);

    std::vector<Coord> comp;
    while (!q.empty()) {
      Coord u = q.front(); q.pop();
      comp.push_back(u);
      for (auto v : neighbors8(H, W, u)) {
        if (frontier_set.count(v) && !visited.count(v)) {
          visited.insert(v);
          q.push(v);
        }
      }
    }
    clusters.push_back(comp);
  }

  std::sort(clusters.begin(), clusters.end(),
            [](const std::vector<Coord>& a, const std::vector<Coord>& b){ return a.size() > b.size(); });
  return clusters;
}

static std::vector<std::vector<double>> bfs_dist_free(const std::vector<std::vector<int>>& grid, Coord start) {
  const int H = (int)grid.size();
  const int W = (int)grid[0].size();
  const double INF = std::numeric_limits<double>::infinity();

  std::vector<std::vector<double>> dist(H, std::vector<double>(W, INF));
  if (!is_free(grid[start.first][start.second])) return dist;

  std::queue<Coord> q;
  q.push(start);
  dist[start.first][start.second] = 0.0;

  while (!q.empty()) {
    Coord u = q.front(); q.pop();
    double du = dist[u.first][u.second];
    for (auto v : neighbors4(H, W, u)) {
      if (!is_free(grid[v.first][v.second])) continue;
      if (!std::isfinite(dist[v.first][v.second])) {
        dist[v.first][v.second] = du + 1.0;
        q.push(v);
      }
    }
  }
  return dist;
}

static Coord rounded_centroid(const std::vector<Coord>& comp) {
  double sr = 0.0, sc = 0.0;
  for (auto c : comp) { sr += c.first; sc += c.second; }
  double cr = sr / std::max(1.0, (double)comp.size());
  double cc = sc / std::max(1.0, (double)comp.size());
  return {(int)std::lround(cr), (int)std::lround(cc)};
}

static Coord choose_goal_near_centroid(const std::vector<std::vector<int>>& grid,
                                      const std::vector<std::vector<double>>& dist,
                                      const std::vector<Coord>& comp,
                                      int max_radius = 6) {
  const int H = (int)grid.size();
  const int W = (int)grid[0].size();
  const double INF = std::numeric_limits<double>::infinity();

  Coord c0 = rounded_centroid(comp);
  Coord best = {-1, -1};
  double best_d = INF;

  for (int rad = 0; rad <= max_radius; ++rad) {
    for (int r = c0.first - rad; r <= c0.first + rad; ++r) {
      for (int c = c0.second - rad; c <= c0.second + rad; ++c) {
        if (!in_bounds(H, W, r, c)) continue;
        if (!is_free(grid[r][c])) continue;
        double d = dist[r][c];
        if (!std::isfinite(d)) continue;
        if (d < best_d) { best_d = d; best = {r,c}; }
      }
    }
    if (best.first != -1) return best;
  }
  return best;
}

static double score_cluster(double distance, int size, double alpha = 1.0, double beta = 2.5) {
  return alpha * distance - beta * (double)size;
}

int main() {
  const int H = 30, W = 40;
  std::vector<std::vector<int>> grid(H, std::vector<int>(W, -1));

  // Known free region
  for (int r = 5; r < 15; ++r) for (int c = 5; c < 18; ++c) grid[r][c] = 0;
  // Obstacles
  for (int r = 5; r < 15; ++r) grid[r][12] = 100;
  for (int c = 12; c < 30; ++c) grid[10][c] = 100;
  // Another known pocket
  for (int r = 18; r < 25; ++r) for (int c = 25; c < 35; ++c) grid[r][c] = 0;

  Coord robot = {8, 8};

  auto frontiers = detect_frontiers(grid);
  auto clusters  = cluster_frontiers(grid, frontiers);
  auto dist = bfs_dist_free(grid, robot);

  Coord best_goal = {-1,-1};
  double best_cost = std::numeric_limits<double>::infinity();

  for (auto& comp : clusters) {
    if ((int)comp.size() < 6) continue;
    Coord goal = choose_goal_near_centroid(grid, dist, comp, 6);
    if (goal.first == -1) continue;
    double d = dist[goal.first][goal.second];
    double J = score_cluster(d, (int)comp.size(), 1.0, 2.5);
    if (J < best_cost) { best_cost = J; best_goal = goal; }
  }

  std::cout << "robot=(" << robot.first << "," << robot.second << ")\n";
  std::cout << "frontier_clusters=" << clusters.size() << "\n";
  std::cout << "best_goal=(" << best_goal.first << "," << best_goal.second << "), cost=" << best_cost << "\n";
  return 0;
}
