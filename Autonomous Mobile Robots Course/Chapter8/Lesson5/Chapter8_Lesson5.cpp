// Chapter8_Lesson5.cpp
// Autonomous Mobile Robots — Chapter 8 (Particle-Filter Localization)
// Lesson 5 Lab: Implement MCL on a Map
//
// Minimal C++17 MCL on an occupancy grid. Writes CSV trajectories.
//
// Build:
//   g++ -O2 -std=c++17 Chapter8_Lesson5.cpp -o mcl_demo
// Run:
//   ./mcl_demo

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <utility>
#include <vector>
using std::vector;

static constexpr double PI = 3.14159265358979323846;

static double wrap_angle(double a) {
  a = std::fmod(a + PI, 2.0 * PI);
  if (a < 0) a += 2.0 * PI;
  return a - PI;
}

struct GridMap {
  int W = 220, H = 160;
  double res = 0.05;
  vector<uint8_t> occ; // 1 occupied, 0 free

  GridMap() : occ(W * H, 0) {}

  uint8_t at(int x, int y) const {
    if (x < 0 || x >= W || y < 0 || y >= H) return 1;
    return occ[y * W + x];
  }

  void set_rect(int x0, int y0, int x1, int y1) {
    for (int y = y0; y < y1; ++y)
      for (int x = x0; x < x1; ++x)
        occ[y * W + x] = 1;
  }

  void build_demo() {
    // outer walls
    for (int x = 0; x < W; ++x) { occ[0*W + x] = 1; occ[(H-1)*W + x] = 1; }
    for (int y = 0; y < H; ++y) { occ[y*W + 0] = 1; occ[y*W + (W-1)] = 1; }
    // obstacles
    set_rect(40, 30, 160, 55);
    set_rect(20, 85, 90, 110);
    set_rect(140, 85, 170, 140);
    set_rect(180, 15, 205, 40);
  }

  void world_to_grid(double wx, double wy, int &gx, int &gy) const {
    gx = static_cast<int>(wx / res);
    gy = static_cast<int>(wy / res);
  }
};

static double ray_cast(const GridMap &m, double x, double y, double th, double rmax, double step = 0.02) {
  double r = 0.0;
  while (r < rmax) {
    double wx = x + r * std::cos(th);
    double wy = y + r * std::sin(th);
    int gx, gy;
    m.world_to_grid(wx, wy, gx, gy);
    if (m.at(gx, gy)) return r;
    r += step;
  }
  return rmax;
}

struct Particle { double x, y, th, w; };

static vector<int> systematic_resample(const vector<double> &w, std::mt19937 &gen) {
  int N = static_cast<int>(w.size());
  std::uniform_real_distribution<double> uni(0.0, 1.0 / N);
  double r0 = uni(gen);
  vector<double> cdf(N);
  double s = 0.0;
  for (int i = 0; i < N; ++i) { s += w[i]; cdf[i] = s; }
  vector<int> idx(N);
  int j = 0;
  for (int i = 0; i < N; ++i) {
    double u = r0 + static_cast<double>(i) / N;
    while (j < N-1 && u > cdf[j]) j++;
    idx[i] = j;
  }
  return idx;
}

struct MCL {
  GridMap m;
  int N = 1200;
  double rmax = 6.0;
  vector<double> beams;
  double sigma_hit = 0.20;
  double z_hit = 0.85, z_rand = 0.15;
  double a1=0.03,a2=0.01,a3=0.03,a4=0.01;

  std::mt19937 gen{1};
  std::normal_distribution<double> n01{0.0, 1.0};

  vector<Particle> P;

  MCL() : P(N) {
    for (int i = 0; i < 13; ++i) {
      double deg = -90.0 + 180.0 * (static_cast<double>(i)/12.0);
      beams.push_back(deg * PI / 180.0);
    }
  }

  void init_uniform() {
    vector<std::pair<int,int>> free;
    free.reserve(m.W*m.H);
    for (int y = 0; y < m.H; ++y)
      for (int x = 0; x < m.W; ++x)
        if (!m.at(x,y)) free.emplace_back(x,y);

    std::uniform_int_distribution<int> pick(0, static_cast<int>(free.size())-1);
    std::uniform_real_distribution<double> uth(-PI, PI);

    for (int i = 0; i < N; ++i) {
      auto [gx, gy] = free[pick(gen)];
      P[i].x = (gx + 0.5) * m.res;
      P[i].y = (gy + 0.5) * m.res;
      P[i].th = uth(gen);
      P[i].w = 1.0 / N;
    }
  }

  void motion_update(double dr1, double dt, double dr2) {
    double sr1 = std::sqrt(a1*dr1*dr1 + a2*dt*dt);
    double st  = std::sqrt(a3*dt*dt  + a4*(dr1*dr1 + dr2*dr2));
    double sr2 = std::sqrt(a1*dr2*dr2 + a2*dt*dt);

    for (auto &p : P) {
      double d1 = dr1 + n01(gen)*sr1;
      double dT = dt  + n01(gen)*st;
      double d2 = dr2 + n01(gen)*sr2;

      double xn = p.x + dT * std::cos(p.th + d1);
      double yn = p.y + dT * std::sin(p.th + d1);
      double thn = wrap_angle(p.th + d1 + d2);

      int gx, gy;
      m.world_to_grid(xn, yn, gx, gy);
      if (!m.at(gx, gy)) { p.x=xn; p.y=yn; p.th=thn; }
    }
  }

  void sensor_update(const vector<double> &z) {
    double sigma = sigma_hit;
    double invs2 = 1.0/(sigma*sigma);
    double norm  = 1.0/(std::sqrt(2.0*PI)*sigma);
    double unif  = 1.0/rmax;

    vector<double> logw(N, 0.0);
    for (size_t bi = 0; bi < beams.size(); ++bi) {
      for (int i = 0; i < N; ++i) {
        const auto &p = P[i];
        double zexp = ray_cast(m, p.x, p.y, p.th + beams[bi], rmax);
        double dz = z[bi] - zexp;
        double prob = z_hit*norm*std::exp(-0.5*dz*dz*invs2) + z_rand*unif;
        logw[i] += std::log(prob + 1e-12);
      }
    }
    double mx = logw[0];
    for (int i = 1; i < N; ++i) mx = std::max(mx, logw[i]);
    vector<double> w(N);
    double s = 0.0;
    for (int i = 0; i < N; ++i) { w[i] = std::exp(logw[i]-mx); s += w[i]; }
    for (int i = 0; i < N; ++i) P[i].w = (s > 0.0) ? (w[i]/s) : (1.0/N);
  }

  double neff() const {
    double s = 0.0;
    for (const auto &p : P) s += p.w*p.w;
    return 1.0/s;
  }

  void resample_if_needed(double ratio=0.55, double inject=0.03) {
    if (neff() >= ratio * N) return;
    vector<double> w(N);
    for (int i = 0; i < N; ++i) w[i] = P[i].w;
    auto idx = systematic_resample(w, gen);
    vector<Particle> P2(N);
    for (int i = 0; i < N; ++i) { P2[i] = P[idx[i]]; P2[i].w = 1.0/N; }
    P.swap(P2);

    int k = static_cast<int>(std::round(inject * N));
    if (k <= 0) return;

    vector<std::pair<int,int>> free;
    for (int y = 0; y < m.H; ++y)
      for (int x = 0; x < m.W; ++x)
        if (!m.at(x,y)) free.emplace_back(x,y);

    std::uniform_int_distribution<int> pick(0, static_cast<int>(free.size())-1);
    std::uniform_real_distribution<double> uth(-PI, PI);

    for (int i = 0; i < k; ++i) {
      auto [gx, gy] = free[pick(gen)];
      P[i].x = (gx + 0.5) * m.res;
      P[i].y = (gy + 0.5) * m.res;
      P[i].th = uth(gen);
    }
  }

  void estimate(double &x, double &y, double &th) const {
    double mx=0,my=0,s=0,c=0;
    for (const auto &p : P) {
      mx += p.w*p.x; my += p.w*p.y;
      s += p.w*std::sin(p.th); c += p.w*std::cos(p.th);
    }
    x=mx; y=my; th=std::atan2(s,c);
  }
};

static void simulate_step(double &x, double &y, double &th, double v, double w, double dt) {
  x += v*std::cos(th)*dt;
  y += v*std::sin(th)*dt;
  th = wrap_angle(th + w*dt);
}

static void odom_from_true(double x1,double y1,double th1, double x2,double y2,double th2,
                           double &dr1,double &dt,double &dr2) {
  double dx = x2-x1, dy=y2-y1;
  dt = std::sqrt(dx*dx + dy*dy);
  double dir = std::atan2(dy, dx);
  dr1 = (dt>1e-9) ? wrap_angle(dir - th1) : 0.0;
  dr2 = wrap_angle(th2 - th1 - dr1);
}

static vector<double> range_scan(const GridMap &m, double x,double y,double th,
                                 const vector<double> &beams, double rmax,
                                 double noise_sigma, std::mt19937 &gen) {
  std::normal_distribution<double> n(0.0, noise_sigma);
  vector<double> z(beams.size());
  for (size_t i = 0; i < beams.size(); ++i) {
    double r = ray_cast(m, x, y, th + beams[i], rmax);
    r += n(gen);
    if (r < 0.0) r = 0.0;
    if (r > rmax) r = rmax;
    z[i] = r;
  }
  return z;
}

int main() {
  MCL pf;
  pf.m.build_demo();
  pf.init_uniform();

  double tx=2.0, ty=2.0, tth=0.0;
  const double dt = 0.25;
  const int T = 180;
  double vCmd=0.35, wCmd=0.25;

  std::mt19937 gen(7);

  std::ofstream ftrue("true_traj.csv");
  std::ofstream fest("est_traj.csv");
  ftrue << "t,x,y,theta\n";
  fest  << "t,x,y,theta\n";

  for (int t=0; t<T; ++t) {
    double nx=tx, ny=ty, nth=tth;
    simulate_step(nx, ny, nth, vCmd, wCmd, dt);

    int gx, gy;
    pf.m.world_to_grid(nx, ny, gx, gy);
    if (pf.m.at(gx, gy)) {
      nx=tx; ny=ty; nth=tth;
      simulate_step(nx, ny, nth, 0.0, 0.8, dt);
    }

    double dr1,dtr,dr2;
    odom_from_true(tx,ty,tth, nx,ny,nth, dr1,dtr,dr2);

    auto z = range_scan(pf.m, nx, ny, nth, pf.beams, pf.rmax, 0.05, gen);

    pf.motion_update(dr1, dtr, dr2);
    pf.sensor_update(z);
    pf.resample_if_needed();

    double ex, ey, eth;
    pf.estimate(ex, ey, eth);

    tx=nx; ty=ny; tth=nth;
    ftrue << t << "," << tx << "," << ty << "," << tth << "\n";
    fest  << t << "," << ex << "," << ey << "," << eth << "\n";
  }

  std::cout << "Wrote true_traj.csv and est_traj.csv\n";
  return 0;
}
