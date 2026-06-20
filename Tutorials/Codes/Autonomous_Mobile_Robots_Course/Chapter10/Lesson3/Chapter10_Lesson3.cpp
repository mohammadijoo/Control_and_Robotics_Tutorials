// Chapter10_Lesson3.cpp
/*
Correlative Scan Matching (2D) — minimal C++17 reference implementation.

This example:
  - builds a binary occupancy grid map (square room + pillar),
  - builds a simple likelihood field by "softening" occupied cells (no SciPy here),
  - runs a coarse-to-fine discrete search around an initial guess pose.

For production use, you would typically:
  - precompute a likelihood field using an exact distance transform,
  - use a branch-and-bound strategy (fast correlative scan matching),
  - integrate with your odometry/IMU prior and your mapping stack.

Build (example):
  g++ -O2 -std=c++17 Chapter10_Lesson3.cpp -o csm

No external dependencies.
*/

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

struct Pose2 {
  double x{0}, y{0}, th{0};
};

struct GridMeta {
  double res{0.05};
};

static inline double clamp(double v, double a, double b) {
  return std::max(a, std::min(b, v));
}

std::vector<uint8_t> makeOcc(int H, int W) {
  std::vector<uint8_t> occ(H*W, 0);
  auto at = [&](int r, int c) -> uint8_t& { return occ[r*W + c]; };

  // walls
  for (int c=0;c<W;c++) { at(0,c)=1; at(H-1,c)=1; }
  for (int r=0;r<H;r++) { at(r,0)=1; at(r,W-1)=1; }

  // pillar
  int cx=W/2, cy=H/2;
  int r = (int)std::round(0.4/0.05);
  for (int yy=cy-r; yy<=cy+r; yy++)
    for (int xx=cx-r; xx<=cx+r; xx++)
      if (yy>=0 && yy<H && xx>=0 && xx<W) at(yy,xx)=1;

  return occ;
}

std::vector<float> softenOccToField(const std::vector<uint8_t>& occ, int H, int W) {
  // Simple local "softening": score is max of occupied in a small radius
  // (a crude proxy for a likelihood field).
  std::vector<float> field(H*W, 0.0f);
  auto occAt = [&](int r,int c)->uint8_t{ return occ[r*W+c]; };
  auto fAt = [&](int r,int c)->float&{ return field[r*W+c]; };

  const int rad = 4; // cells
  for (int r=0;r<H;r++){
    for (int c=0;c<W;c++){
      float best = 0.0f;
      for (int dr=-rad; dr<=rad; dr++){
        for (int dc=-rad; dc<=rad; dc++){
          int rr=r+dr, cc=c+dc;
          if (rr<0||rr>=H||cc<0||cc>=W) continue;
          if (occAt(rr,cc)){
            float d2 = float(dr*dr + dc*dc);
            float s = std::exp(-d2 / (2.0f*float(rad*rad)));
            if (s>best) best=s;
          }
        }
      }
      fAt(r,c)=best;
    }
  }
  return field;
}

void transformPoints(const Pose2& p, const std::vector<std::array<double,2>>& pts_r,
                     std::vector<std::array<double,2>>& pts_w) {
  const double c = std::cos(p.th), s = std::sin(p.th);
  pts_w.resize(pts_r.size());
  for (size_t i=0;i<pts_r.size();i++){
    const double xr = pts_r[i][0], yr = pts_r[i][1];
    const double xw = c*xr - s*yr + p.x;
    const double yw = s*xr + c*yr + p.y;
    pts_w[i] = {xw, yw};
  }
}

double scorePose(const std::vector<float>& field, int H, int W, double res,
                 const Pose2& p, const std::vector<std::array<double,2>>& pts_r) {
  std::vector<std::array<double,2>> pts_w;
  transformPoints(p, pts_r, pts_w);

  double s = 0.0;
  for (auto& q : pts_w){
    int gx = (int)std::floor(q[0] / res);
    int gy = (int)std::floor(q[1] / res);
    if (gx>=0 && gx<W && gy>=0 && gy<H){
      s += field[gy*W + gx];
    }
  }
  return s;
}

std::vector<float> downsampleMax(const std::vector<float>& g, int H, int W, int& H2, int& W2){
  H2 = H/2; W2 = W/2;
  std::vector<float> out(H2*W2, 0.0f);
  auto atIn = [&](int r,int c)->float{ return g[r*W+c]; };
  auto atOut = [&](int r,int c)->float&{ return out[r*W2+c]; };
  for(int r=0;r<H2;r++){
    for(int c=0;c<W2;c++){
      float m = 0.0f;
      m = std::max(m, atIn(2*r,2*c));
      m = std::max(m, atIn(2*r+1,2*c));
      m = std::max(m, atIn(2*r,2*c+1));
      m = std::max(m, atIn(2*r+1,2*c+1));
      atOut(r,c)=m;
    }
  }
  return out;
}

struct Cand { Pose2 p; double score; };

int main(){
  GridMeta meta; meta.res = 0.05;
  const double size_m = 10.0;
  const int W = (int)std::round(size_m / meta.res);
  const int H = W;

  auto occ = makeOcc(H,W);
  auto field0 = softenOccToField(occ,H,W);

  // pyramid
  int H1,W1,H2,W2;
  auto field1 = downsampleMax(field0,H,W,H1,W1);
  auto field2 = downsampleMax(field1,H1,W1,H2,W2);

  // "true" pose and a synthetic scan: sample occupied points in range (no raycast)
  Pose2 ptrue{2.4, 3.2, 18.0*M_PI/180.0};
  std::vector<std::array<double,2>> occ_pts_w;
  for(int r=0;r<H;r+=2){
    for(int c=0;c<W;c+=2){
      if (occ[r*W+c]){
        occ_pts_w.push_back({(c+0.5)*meta.res, (r+0.5)*meta.res});
      }
    }
  }
  // convert to robot frame
  const double ct = std::cos(ptrue.th), st = std::sin(ptrue.th);
  std::vector<std::array<double,2>> pts_r;
  pts_r.reserve(800);
  std::mt19937 rng(7);
  std::normal_distribution<double> n01(0.0, 0.01);
  for (auto& pw : occ_pts_w){
    double dx = pw[0]-ptrue.x;
    double dy = pw[1]-ptrue.y;
    double xr =  ct*dx + st*dy;
    double yr = -st*dx + ct*dy;
    double d = std::sqrt(xr*xr + yr*yr);
    if (d>0.3 && d<6.0){
      pts_r.push_back({xr + n01(rng), yr + n01(rng)});
    }
  }
  // subsample
  std::shuffle(pts_r.begin(), pts_r.end(), rng);
  if ((int)pts_r.size()>600) pts_r.resize(600);

  Pose2 p0{2.55, 3.05, 10.0*M_PI/180.0};

  auto search_level = [&](const std::vector<float>& field, int HH, int WW, double res_level,
                          const std::vector<Cand>& seeds,
                          double win_xy, double win_th, double step_xy, double step_th, int topK){
    std::vector<Cand> cands;
    for (auto& s : seeds){
      for (double x = s.p.x - win_xy; x <= s.p.x + win_xy + 1e-12; x += step_xy){
        for (double y = s.p.y - win_xy; y <= s.p.y + win_xy + 1e-12; y += step_xy){
          for (double th = s.p.th - win_th; th <= s.p.th + win_th + 1e-12; th += step_th){
            Pose2 p{x,y,th};
            double sc = scorePose(field, HH, WW, res_level, p, pts_r);
            cands.push_back({p, sc});
          }
        }
      }
    }
    std::partial_sort(cands.begin(), cands.begin() + std::min(topK,(int)cands.size()), cands.end(),
                      [](const Cand& a, const Cand& b){ return a.score > b.score; });
    if ((int)cands.size()>topK) cands.resize(topK);
    return cands;
  };

  std::vector<Cand> seeds{{p0, 0.0}};

  // coarsest -> finest
  seeds = search_level(field2, H2, W2, meta.res*4.0, seeds, 0.6, 12.0*M_PI/180.0,
                       meta.res*4.0, 2.0*M_PI/180.0, 50);
  seeds = search_level(field1, H1, W1, meta.res*2.0, seeds, 0.3, 6.0*M_PI/180.0,
                       meta.res*2.0, 1.0*M_PI/180.0, 50);
  seeds = search_level(field0, H, W, meta.res, seeds, 0.15, 3.0*M_PI/180.0,
                       meta.res, 0.5*M_PI/180.0, 50);

  auto best = *std::max_element(seeds.begin(), seeds.end(), [](const Cand& a, const Cand& b){
    return a.score < b.score;
  });

  std::cout << "True pose:     x=" << ptrue.x << " y=" << ptrue.y
            << " th(deg)=" << ptrue.th*180.0/M_PI << "\n";
  std::cout << "Initial guess: x=" << p0.x << " y=" << p0.y
            << " th(deg)=" << p0.th*180.0/M_PI << "\n";
  std::cout << "Matched pose:  x=" << best.p.x << " y=" << best.p.y
            << " th(deg)=" << best.p.th*180.0/M_PI
            << " score=" << best.score << "\n";
  return 0;
}
