#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include <unordered_map>
#include <tuple>

struct State {
  int ix, iy, kth;
};

struct Node {
  State s;
  double g;
  double f;
};

struct NodeCmp {
  bool operator()(const Node& a, const Node& b) const {
    return a.f > b.f; // min-heap via >
  }
};

struct StateHash {
  std::size_t operator()(State const& s) const noexcept {
    return ((s.ix * 73856093) ^ (s.iy * 19349663) ^ (s.kth * 83492791));
  }
};

struct StateEq {
  bool operator()(State const& a, State const& b) const noexcept {
    return a.ix == b.ix && a.iy == b.iy && a.kth == b.kth;
  }
};

static const int NX = 50;
static const int NY = 30;
static const int N_THETA = 16;
static const double DX = 1.0;
static const double DY = 1.0;
static const double L = 2.0;
static const double V = 1.0;
static const double DT = 1.0;

// A tiny occupancy grid for demo
int occ_grid[NX][NY] = {0};

inline bool in_bounds(int ix, int iy) {
  return 0 <= ix && ix < NX && 0 <= iy && iy < NY;
}

inline bool is_free(int ix, int iy) {
  return in_bounds(ix, iy) && occ_grid[ix][iy] == 0;
}

int angle_index(double theta) {
  double two_pi = 2.0 * M_PI;
  theta = fmod(theta + two_pi, two_pi);
  int k = static_cast<int>(std::round(theta / two_pi * N_THETA)) % N_THETA;
  return k;
}

double angle_from_index(int k) {
  return 2.0 * M_PI * k / N_THETA;
}

struct Primitive {
  double dx_ref;
  double dy_ref;
  double dth_ref;
  double length;
};

std::vector<Primitive> make_primitives() {
  std::vector<Primitive> prims;
  double steer[3] = {-0.4, 0.0, 0.4};
  for (int i = 0; i < 3; ++i) {
    double phi = steer[i];
    double kappa = std::tan(phi) / L;
    Primitive p;
    if (std::fabs(kappa) < 1e-6) {
      p.dx_ref = V * DT;
      p.dy_ref = 0.0;
      p.dth_ref = 0.0;
      p.length = V * DT;
    } else {
      p.dth_ref = V * kappa * DT;
      double th_end = p.dth_ref;
      p.dx_ref = (1.0 / kappa) * std::sin(th_end);
      p.dy_ref = -(1.0 / kappa) * (std::cos(th_end) - 1.0);
      p.length = std::fabs(p.dth_ref / kappa);
    }
    prims.push_back(p);
  }
  return prims;
}

double heuristic(State const& s, State const& goal) {
  double x = s.ix * DX;
  double y = s.iy * DY;
  double xg = goal.ix * DX;
  double yg = goal.iy * DY;
  return std::hypot(x - xg, y - yg);
}

int main() {
  std::vector<Primitive> prims = make_primitives();

  State start{2, 2, angle_index(0.0)};
  State goal{40, 20, angle_index(0.0)};

  std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
  std::unordered_map<State, double, StateHash, StateEq> g;
  std::unordered_map<State, State, StateHash, StateEq> parent;

  g[start] = 0.0;
  parent[start] = start;
  open.push(Node{start, 0.0, heuristic(start, goal)});

  std::unordered_map<State, bool, StateHash, StateEq> closed;

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();

    if (closed[cur.s]) continue;
    closed[cur.s] = true;

    if (cur.s.ix == goal.ix && cur.s.iy == goal.iy) {
      // Reconstruct and output path if desired
      break;
    }

    double theta = angle_from_index(cur.s.kth);
    for (auto const& p : prims) {
      double gx = cur.s.ix * DX;
      double gy = cur.s.iy * DY;
      double dx = std::cos(theta) * p.dx_ref - std::sin(theta) * p.dy_ref;
      double dy = std::sin(theta) * p.dx_ref + std::cos(theta) * p.dy_ref;
      double gx2 = gx + dx;
      double gy2 = gy + dy;
      double th2 = theta + p.dth_ref;

      State s2;
      s2.ix = static_cast<int>(std::round(gx2 / DX));
      s2.iy = static_cast<int>(std::round(gy2 / DY));
      s2.kth = angle_index(th2);

      if (!is_free(s2.ix, s2.iy)) continue;

      double new_g = g[cur.s] + p.length;
      auto it = g.find(s2);
      if (it == g.end() || new_g < it->second) {
        g[s2] = new_g;
        parent[s2] = cur.s;
        double f = new_g + heuristic(s2, goal);
        open.push(Node{s2, new_g, f});
      }
    }
  }

  return 0;
}
      
