// Chapter17_Lesson5.cpp
/*
Autonomous Mobile Robots — Chapter 17 (Exploration and Active Mapping)
Lesson 5 Lab: Autonomous Exploration in Unknown Map (single robot)

A minimal C++17 grid-world exploration demo:
- Log-odds occupancy belief
- Frontier detection
- Candidate selection using IG - cost - risk
- One-step motion + sensing update (ray casting)

Build:
  g++ -std=c++17 -O2 Chapter17_Lesson5.cpp -o explore

Run:
  ./explore

Note: For real robots, integrate frontier selection with Nav2/ROS2 and a SLAM map.
*/

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <limits>
#include <unordered_map>
#include <algorithm>

struct Pose2D { int x; int y; };

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
static double logit(double p) {
  p = clamp(p, 1e-6, 1.0 - 1e-6);
  return std::log(p / (1.0 - p));
}
static double sigmoid(double l) {
  if (l >= 0.0) { double z = std::exp(-l); return 1.0 / (1.0 + z); }
  double z = std::exp(l); return z / (1.0 + z);
}

static double bernoulli_entropy(double p) {
  p = clamp(p, 1e-12, 1.0 - 1e-12);
  return -p * std::log(p) - (1.0 - p) * std::log(1.0 - p);
}

struct BeliefGrid {
  int w, h;
  std::vector<double> l;
  double l_occ, l_free, l_min, l_max;

  BeliefGrid(int W, int H, double p_occ=0.7, double p_free=0.3) : w(W), h(H), l(W*H, 0.0) {
    l_occ = logit(p_occ);
    l_free = logit(p_free);
    l_min = logit(0.02);
    l_max = logit(0.98);
  }

  inline int idx(int x, int y) const { return y*w + x; }

  int classify(int x, int y, double p_occ_th=0.65, double p_free_th=0.35) const {
    double p = sigmoid(l[idx(x,y)]);
    if (p >= p_occ_th) return 1;
    if (p <= p_free_th) return 0;
    return -1;
  }

  double prob(int x, int y) const { return sigmoid(l[idx(x,y)]); }

  void update_free(int x, int y) {
    int i = idx(x,y);
    l[i] = clamp(l[i] + l_free, l_min, l_max);
  }
  void update_occ(int x, int y) {
    int i = idx(x,y);
    l[i] = clamp(l[i] + l_occ, l_min, l_max);
  }

  double total_entropy() const {
    double s = 0.0;
    for (double li : l) {
      double p = sigmoid(li);
      s += bernoulli_entropy(p);
    }
    return s;
  }
};

struct World {
  int w, h;
  std::vector<uint8_t> occ;

  World(int W, int H, double obstacle_prob=0.22, uint32_t seed=7) : w(W), h(H), occ(W*H, 0) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    for (int y=0;y<h;y++){
      for (int x=0;x<w;x++){
        occ[y*w+x] = (U(rng) < obstacle_prob) ? 1 : 0;
      }
    }
    for (int x=0;x<w;x++){ occ[x]=1; occ[(h-1)*w+x]=1; }
    for (int y=0;y<h;y++){ occ[y*w]=1; occ[y*w+(w-1)]=1; }
  }

  inline bool is_occ(int x, int y) const { return occ[y*w+x] != 0; }

  static std::vector<std::pair<int,int>> bresenham(int x0,int y0,int x1,int y1){
    std::vector<std::pair<int,int>> cells;
    int dx = std::abs(x1-x0);
    int dy = -std::abs(y1-y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x=x0, y=y0;
    while(true){
      cells.push_back({x,y});
      if(x==x1 && y==y1) break;
      int e2 = 2*err;
      if(e2 >= dy){ err += dy; x += sx; }
      if(e2 <= dx){ err += dx; y += sy; }
    }
    return cells;
  }

  void sense_update(const Pose2D& pose, BeliefGrid& belief, int n_rays=48, int r_max=10) const {
    double fov = 2.0*M_PI;
    double start = -fov/2.0;
    for(int k=0;k<n_rays;k++){
      double theta = start + (double)k / std::max(1, n_rays-1) * fov;
      int x1 = (int)std::llround(pose.x + r_max*std::cos(theta));
      int y1 = (int)std::llround(pose.y + r_max*std::sin(theta));
      x1 = (int)clamp(x1, 0, w-1);
      y1 = (int)clamp(y1, 0, h-1);
      auto line = bresenham(pose.x, pose.y, x1, y1);
      for(size_t i=1;i<line.size();i++){
        int cx = line[i].first, cy = line[i].second;
        if(is_occ(cx,cy)){ belief.update_occ(cx,cy); break; }
        belief.update_free(cx,cy);
      }
    }
  }
};

static inline void neighbors4(int x,int y,std::vector<std::pair<int,int>>& out){
  out.clear();
  out.push_back({x+1,y}); out.push_back({x-1,y}); out.push_back({x,y+1}); out.push_back({x,y-1});
}

static std::vector<std::pair<int,int>> frontier_cells(const BeliefGrid& belief){
  std::vector<std::pair<int,int>> fronts;
  std::vector<std::pair<int,int>> neigh;
  for(int y=1;y<belief.h-1;y++){
    for(int x=1;x<belief.w-1;x++){
      if(belief.classify(x,y) != -1) continue;
      neighbors4(x,y,neigh);
      for(auto [nx,ny]:neigh){
        if(belief.classify(nx,ny) == 0){ fronts.push_back({x,y}); break; }
      }
    }
  }
  return fronts;
}

static double approx_ig(const BeliefGrid& belief, const Pose2D& pose, int r_max=10){
  // IG ~ (#unknown within disk)*ln(2)
  int unknown = 0;
  int r2 = r_max*r_max;
  for(int y=std::max(0, pose.y-r_max); y<std::min(belief.h, pose.y+r_max+1); y++){
    for(int x=std::max(0, pose.x-r_max); x<std::min(belief.w, pose.x+r_max+1); x++){
      int dx=x-pose.x, dy=y-pose.y;
      if(dx*dx + dy*dy <= r2){
        if(belief.classify(x,y) == -1) unknown++;
      }
    }
  }
  return unknown * std::log(2.0);
}

struct Node { double d; int x; int y; };

static double dijkstra_cost(const BeliefGrid& belief, int sx,int sy,int gx,int gy,
                            std::vector<int>& parent){
  const double INF = 1e18;
  parent.assign(belief.w*belief.h, -1);
  std::vector<double> dist(belief.w*belief.h, INF);
  auto idx = [&](int x,int y){ return y*belief.w + x; };

  auto cmp = [](const Node& a,const Node& b){ return a.d > b.d; };
  std::priority_queue<Node,std::vector<Node>,decltype(cmp)> pq(cmp);

  dist[idx(sx,sy)] = 0.0;
  pq.push({0.0, sx, sy});
  std::vector<std::pair<int,int>> neigh;

  while(!pq.empty()){
    Node cur = pq.top(); pq.pop();
    int i = idx(cur.x, cur.y);
    if(cur.d != dist[i]) continue;
    if(cur.x==gx && cur.y==gy) return cur.d;

    neighbors4(cur.x, cur.y, neigh);
    for(auto [nx,ny]:neigh){
      if(nx<0||nx>=belief.w||ny<0||ny>=belief.h) continue;
      if(belief.classify(nx,ny) == 1) continue;
      double step = (belief.classify(nx,ny) == 0) ? 1.0 : 3.0;
      double nd = cur.d + step;
      int j = idx(nx,ny);
      if(nd < dist[j]){
        dist[j] = nd;
        parent[j] = i;
        pq.push({nd,nx,ny});
      }
    }
  }
  return INF;
}

static std::vector<std::pair<int,int>> recover_path(const BeliefGrid& belief,
                                                    const std::vector<int>& parent,
                                                    int sx,int sy,int gx,int gy){
  auto idx = [&](int x,int y){ return y*belief.w + x; };
  int s = idx(sx,sy);
  int g = idx(gx,gy);
  std::vector<std::pair<int,int>> path;
  if(s==g){ path.push_back({sx,sy}); return path; }
  if(parent[g] < 0) return path;
  int cur = g;
  while(cur != s && cur >= 0){
    int x = cur % belief.w;
    int y = cur / belief.w;
    path.push_back({x,y});
    cur = parent[cur];
  }
  path.push_back({sx,sy});
  std::reverse(path.begin(), path.end());
  return path;
}

static double path_risk(const BeliefGrid& belief, const std::vector<std::pair<int,int>>& path){
  if(path.empty()) return std::numeric_limits<double>::infinity();
  double s = 0.0;
  for(auto [x,y]: path) s += belief.prob(x,y);
  return s / (double)path.size();
}

int main(){
  int W=60, H=45;
  World world(W,H,0.22,7);
  BeliefGrid belief(W,H,0.72,0.28);

  // start pose: find first free
  Pose2D pose{2,2};
  for(int y=1;y<H-1;y++){
    for(int x=1;x<W-1;x++){
      if(!world.is_occ(x,y)){ pose = {x,y}; goto done; }
    }
  }
done:;

  // weights
  double w_ig=1.0, w_cost=0.35, w_risk=1.2;

  world.sense_update(pose, belief);

  double travel = 0.0;
  int steps = 0;

  for(int it=0; it<2500; it++){
    steps++;
    auto fronts = frontier_cells(belief);
    if(fronts.empty()) break;

    // sample a subset of frontier cells as candidate goals
    std::mt19937 rng(steps*17u + 3u);
    std::shuffle(fronts.begin(), fronts.end(), rng);
    int K = std::min((int)fronts.size(), 60);

    double best_score = -1e18;
    std::pair<int,int> best_goal = fronts[0];
    std::vector<int> best_parent;

    for(int i=0;i<K;i++){
      int gx = fronts[i].first, gy = fronts[i].second;
      std::vector<int> parent;
      double cost = dijkstra_cost(belief, pose.x, pose.y, gx, gy, parent);
      if(!std::isfinite(cost) || cost > 1e17) continue;
      auto path = recover_path(belief, parent, pose.x, pose.y, gx, gy);
      if(path.size() < 2) continue;

      double ig = approx_ig(belief, Pose2D{gx,gy}, 10);
      double risk = path_risk(belief, path);
      double score = w_ig*ig - w_cost*cost - w_risk*risk;

      if(score > best_score){
        best_score = score;
        best_goal = {gx,gy};
        best_parent = parent;
      }
    }

    // move one step toward best_goal
    std::vector<int> parent;
    dijkstra_cost(belief, pose.x, pose.y, best_goal.first, best_goal.second, parent);
    auto path = recover_path(belief, parent, pose.x, pose.y, best_goal.first, best_goal.second);
    if(path.size() < 2) break;
    int nx = path[1].first, ny = path[1].second;

    if(world.is_occ(nx,ny)){
      belief.update_occ(nx,ny);
      continue;
    }
    pose = {nx,ny};
    travel += 1.0;

    world.sense_update(pose, belief);
  }

  // compute known fraction
  int known = 0;
  for(int y=0;y<H;y++){
    for(int x=0;x<W;x++){
      if(belief.classify(x,y) != -1) known++;
    }
  }
  double known_frac = (double)known / (double)(W*H);

  std::cout << "=== Exploration Summary (C++) ===\n";
  std::cout << "steps: " << steps << "\n";
  std::cout << "travel: " << travel << "\n";
  std::cout << "known_fraction: " << known_frac << "\n";
  std::cout << "entropy_final: " << belief.total_entropy() << "\n";
  return 0;
}
