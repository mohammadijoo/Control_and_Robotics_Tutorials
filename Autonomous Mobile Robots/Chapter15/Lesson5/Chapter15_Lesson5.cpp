// Chapter15_Lesson5.cpp
// Lab: Compare Local Planners in Dense Obstacles (DWA vs simplified TEB-proxy)
// Build: g++ -O2 -std=c++17 Chapter15_Lesson5.cpp -o lab
// Run:   ./lab 30 0
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

struct Ob { double x,y,r; };
struct Res { bool success=false, collision=false; double time=0, L=0, cmin=1e9, w2=0; };

static inline double wrap(double a){ a = std::fmod(a + M_PI, 2*M_PI); if(a<0) a+=2*M_PI; return a - M_PI; }
static inline double hypot2(double x,double y){ return std::sqrt(x*x+y*y); }
static inline double corridor_y(double x){ return 0.9*std::sin(0.6*x); }

std::vector<std::array<double,2>> reference_path(int n=120){
  std::vector<std::array<double,2>> p; p.reserve(n);
  double x0=0.5,x1=11.5;
  for(int i=0;i<n;i++){
    double x = x0 + (x1-x0)*double(i)/(n-1);
    p.push_back({x, corridor_y(x)});
  }
  return p;
}

std::vector<Ob> sample_dense_world(int nObs, int seed){
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> Ux(0.5, 11.5), Uy(-2.5, 2.5), Ur(0.12, 0.32);
  std::vector<Ob> obs; obs.reserve(nObs);
  int tries=0;
  while((int)obs.size()<nObs && tries<20000){
    tries++;
    double x=Ux(rng), y=Uy(rng), r=Ur(rng);
    if(std::abs(y - corridor_y(x)) < 0.55 + r) continue;
    bool ok=true;
    for(auto &o: obs){
      double dx=x-o.x, dy=y-o.y;
      double rr=r+o.r+0.05;
      if(dx*dx+dy*dy < rr*rr){ ok=false; break; }
    }
    if(ok) obs.push_back({x,y,r});
  }
  return obs;
}

double clearance(double px,double py, const std::vector<Ob>& obs, double R){
  double d=1e18;
  for(auto &o: obs){
    d = std::min(d, hypot2(px-o.x, py-o.y) - o.r - R);
  }
  return d;
}

std::array<double,3> step_unicycle(const std::array<double,3>& x, double v,double w,double dt){
  return { x[0] + v*std::cos(x[2])*dt,
           x[1] + v*std::sin(x[2])*dt,
           wrap(x[2] + w*dt) };
}

std::pair<std::array<double,3>, double> rollout_min_clear(std::array<double,3> x, double v,double w,
                                                          const std::vector<Ob>& obs, double R, double dt, double T){
  int steps = std::max(1, (int)std::lround(T/dt));
  double mc=1e18;
  for(int i=0;i<steps;i++){
    x = step_unicycle(x,v,w,dt);
    mc = std::min(mc, clearance(x[0],x[1],obs,R));
    if(mc<=0) break;
  }
  return {x, mc};
}

struct Params{
  double R=0.25, vmax=0.9, wmax=1.6, a=1.2, alpha=2.5;
  double dt=0.1, T=2.0, goal_tol=0.25;
  int max_steps=800, n_obs=45;
};

std::pair<double,double> dwa_command(const std::array<double,3>& x, double v0,double w0,
                                     const std::vector<std::array<double,2>>& path,
                                     const std::vector<Ob>& obs, const Params& p){
  double vmin=std::max(0.0, v0 - p.a*p.dt), vmax=std::min(p.vmax, v0 + p.a*p.dt);
  double wmin=std::max(-p.wmax, w0 - p.alpha*p.dt), wmax=std::min(p.wmax, w0 + p.alpha*p.dt);

  double bestJ=-1e18, bestV=0, bestW=0;
  auto goal = path.back();

  for(int i=0;i<9;i++){
    double v = vmin + (vmax-vmin)*double(i)/8.0;
    for(int j=0;j<17;j++){
      double w = wmin + (wmax-wmin)*double(j)/16.0;
      auto rr = rollout_min_clear(x,v,w,obs,p.R,p.dt,p.T);
      double mc = rr.second;
      if(mc <= p.R + 0.05) continue;
      double gx = rr.first[0]-goal[0], gy = rr.first[1]-goal[1];
      double goalDist = hypot2(gx,gy);
      double J = -0.6*goalDist + 1.8*mc + 0.4*(v/(p.vmax+1e-9)) - 0.12*(w*w);
      if(J>bestJ){ bestJ=J; bestV=v; bestW=w; }
    }
  }
  return {bestV, bestW};
}

std::vector<std::array<double,2>> optimize_band(const std::array<double,3>& x,
                                                const std::vector<Ob>& obs, const Params& p,
                                                int N=12, double bandLen=2.6, int iters=20, double step=0.12){
  std::vector<std::array<double,2>> pts(N);
  for(int i=0;i<N;i++){
    double xx = x[0] + bandLen*double(i)/(N-1);
    pts[i] = {xx, corridor_y(xx)};
  }
  pts[0] = {x[0], x[1]};

  for(int it=0; it<iters; ++it){
    std::vector<std::array<double,2>> g(N, {0,0});

    // smoothness
    for(int i=1;i<N-1;i++){
      g[i][0] += 0.35*(2*pts[i][0] - pts[i-1][0] - pts[i+1][0]);
      g[i][1] += 0.35*(2*pts[i][1] - pts[i-1][1] - pts[i+1][1]);
    }

    // obstacle repulsion (nearest obstacle)
    for(int i=1;i<N;i++){
      double bestD=1e18; std::array<double,2> bestU{0,0};
      for(auto &o: obs){
        double vx=pts[i][0]-o.x, vy=pts[i][1]-o.y;
        double n = hypot2(vx,vy) + 1e-12;
        double d = n - o.r - p.R;
        if(d<bestD){ bestD=d; bestU={vx/n, vy/n}; }
      }
      if(bestD < 0.9){
        double phi = std::exp(-4.5*(bestD - 0.9));
        g[i][0] += 1.8*(-4.5*phi)*bestU[0];
        g[i][1] += 1.8*(-4.5*phi)*bestU[1];
      }
    }

    // time preference
    for(int i=1;i<N;i++){
      g[i][0] += 0.25*(pts[i][0]-pts[i-1][0]);
      g[i][1] += 0.25*(pts[i][1]-pts[i-1][1]);
    }

    for(int i=1;i<N;i++){
      pts[i][0] -= step*g[i][0];
      pts[i][1] -= step*g[i][1];
    }
    pts[0] = {x[0], x[1]};
  }
  return pts;
}

std::pair<double,double> teb_command(const std::array<double,3>& x, double v0,double w0,
                                     const std::vector<std::array<double,2>>& path,
                                     const std::vector<Ob>& obs, const Params& p){
  (void)path; // not needed in this proxy
  auto pts = optimize_band(x, obs, p);
  double dx=pts[1][0]-pts[0][0], dy=pts[1][1]-pts[0][1];
  double heading = std::atan2(dy,dx);
  double herr = wrap(heading - x[2]);

  double w = std::clamp(2.2*herr, -p.wmax, p.wmax);
  double v = std::clamp(0.8*(hypot2(dx,dy)/p.dt), 0.0, p.vmax);
  v *= 1.0/(1.0 + 1.2*std::abs(w));

  // accel limits
  v = std::clamp(v, std::max(0.0, v0 - p.a*p.dt), std::min(p.vmax, v0 + p.a*p.dt));
  w = std::clamp(w, w0 - p.alpha*p.dt, w0 + p.alpha*p.dt);
  return {v,w};
}

Res run_one(const std::string& planner, int seed, const Params& p){
  auto obs  = sample_dense_world(p.n_obs, seed);
  auto path = reference_path();
  auto goal = path.back();

  std::array<double,3> x{0.6,0.0,0.0};
  double v=0,w=0;
  Res r; r.time = p.max_steps*p.dt;

  for(int k=0;k<p.max_steps;k++){
    double c = clearance(x[0],x[1],obs,p.R);
    r.cmin = std::min(r.cmin, c);
    if(c<=0){ r.collision=true; r.time=k*p.dt; return r; }

    double gdist = hypot2(x[0]-goal[0], x[1]-goal[1]);
    if(gdist <= p.goal_tol){ r.success=true; r.time=k*p.dt; return r; }

    double vcmd, wcmd;
    if(planner=="teb"){ auto u = teb_command(x,v,w,path,obs,p); vcmd=u.first; wcmd=u.second; }
    else             { auto u = dwa_command(x,v,w,path,obs,p); vcmd=u.first; wcmd=u.second; }

    auto x2 = step_unicycle(x, vcmd, wcmd, p.dt);
    r.L  += hypot2(x2[0]-x[0], x2[1]-x[1]);
    r.w2 += (wcmd*wcmd)*p.dt;
    x=x2; v=vcmd; w=wcmd;
  }
  return r;
}

struct Summary{ int n=0; double succ=0,col=0, t=0,L=0,cmin=0,w2=0; };

Summary summarize(const std::vector<Res>& R){
  Summary s; s.n = (int)R.size();
  for(auto &r: R){
    s.succ += r.success?1:0;
    s.col  += r.collision?1:0;
    s.t    += r.time; s.L += r.L; s.cmin += r.cmin; s.w2 += r.w2;
  }
  s.t/=s.n; s.L/=s.n; s.cmin/=s.n; s.w2/=s.n;
  return s;
}

int main(int argc, char** argv){
  int trials = (argc>1)? std::atoi(argv[1]) : 30;
  int seed0  = (argc>2)? std::atoi(argv[2]) : 0;
  Params p;

  for(auto name: {std::string("dwa"), std::string("teb")}){
    std::vector<Res> R; R.reserve(trials);
    for(int i=0;i<trials;i++) R.push_back(run_one(name, seed0+i, p));
    auto s = summarize(R);
    std::cout << "\n" << (name=="dwa"?"DWA":"TEB") << " trials="<<s.n
              << " success_rate="<< (s.succ/s.n)
              << " collision_rate="<< (s.col/s.n)
              << " time_mean="<< s.t
              << " L_mean="<< s.L
              << " cmin_mean="<< s.cmin
              << " w2_mean="<< s.w2
              << "\n";
  }
  return 0;
}
