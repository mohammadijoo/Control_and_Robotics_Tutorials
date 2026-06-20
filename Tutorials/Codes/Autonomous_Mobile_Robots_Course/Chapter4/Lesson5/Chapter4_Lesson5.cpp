// Chapter4_Lesson5.cpp
// Parameter Effects on Real Navigation (Mobile Robot Dynamics - Applied)
//
// Minimal, dependency-light C++ implementation of the same model used in the lesson.
// It simulates a differential-drive robot with simple v/omega dynamics under a
// nominal PI controller and prints RMS trajectory deviation when physical
// parameters in the plant differ from the controller's nominal parameters.
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter4_Lesson5.cpp -o Chapter4_Lesson5
//
// Note: In production stacks, Eigen (linear algebra) and ROS 2 (rclcpp) are typical.
// Here we avoid external dependencies for portability.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

struct Params {
  double m  = 25.0;   // kg
  double Iz = 2.0;    // kg*m^2
  double b  = 0.45;   // m
  double r  = 0.10;   // m
  double mu = 0.8;    // -
  double cv = 0.4;    // 1/s
  double cw = 0.6;    // 1/s
  double g  = 9.81;   // m/s^2
};

static inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi ? hi : x);
}

struct State {
  double px=0, py=0, th=0, v=0, om=0;
};

static inline void reference(double t, double &vref, double &omref) {
  vref  = 0.9 + 0.2 * std::sin(0.2 * t);
  omref = 0.35 + 0.15 * std::sin(0.17 * t + 0.7);
}

struct PIController {
  Params pnom;
  double kvp=180.0, kvi=35.0, kwp=18.0, kwi=4.0;
  double iv=0.0, iw=0.0;

  explicit PIController(const Params& p) : pnom(p) {}

  void reset() { iv = 0.0; iw = 0.0; }

  void torques(const State& x, double vref, double omref, double dt, double &tauL, double &tauR) {
    double ev = vref - x.v;
    double ew = omref - x.om;
    iv += ev * dt;
    iw += ew * dt;

    double Fcmd = pnom.m  * (kvp * ev + kvi * iv) + pnom.m  * pnom.cv * x.v;
    double Mcmd = pnom.Iz * (kwp * ew + kwi * iw) + pnom.Iz * pnom.cw * x.om;

    double tau_max = 35.0;
    tauR = 0.5 * pnom.r * (Fcmd + 2.0 * Mcmd / pnom.b);
    tauL = 0.5 * pnom.r * (Fcmd - 2.0 * Mcmd / pnom.b);
    tauR = clamp(tauR, -tau_max, tau_max);
    tauL = clamp(tauL, -tau_max, tau_max);
  }
};

static inline State deriv(const State& x, double tauL, double tauR, const Params& p) {
  // Force and moment from torques
  double F  = (tauR + tauL) / p.r;
  double Mz = (p.b / (2.0 * p.r)) * (tauR - tauL);

  // Traction saturation
  double Fmax = p.mu * p.m * p.g;
  F = clamp(F, -Fmax, Fmax);

  State dx;
  dx.px = x.v * std::cos(x.th);
  dx.py = x.v * std::sin(x.th);
  dx.th = x.om;
  dx.v  = (1.0 / p.m) * F - p.cv * x.v;
  dx.om = (1.0 / p.Iz) * Mz - p.cw * x.om;
  return dx;
}

static inline State add(const State& a, const State& b, double s) {
  State c;
  c.px = a.px + s*b.px;
  c.py = a.py + s*b.py;
  c.th = a.th + s*b.th;
  c.v  = a.v  + s*b.v;
  c.om = a.om + s*b.om;
  return c;
}

static inline State rk4_step(const State& x, double tauL, double tauR, double dt, const Params& p) {
  State k1 = deriv(x, tauL, tauR, p);
  State k2 = deriv(add(x, k1, 0.5*dt), tauL, tauR, p);
  State k3 = deriv(add(x, k2, 0.5*dt), tauL, tauR, p);
  State k4 = deriv(add(x, k3, dt),     tauL, tauR, p);
  State xn = x;
  xn = add(xn, k1, dt/6.0);
  xn = add(xn, k2, dt/3.0);
  xn = add(xn, k3, dt/3.0);
  xn = add(xn, k4, dt/6.0);
  return xn;
}

static inline double wrap_unwrap(double prev, double current) {
  // Unwrap heading to avoid 2*pi discontinuity (very simple)
  double d = current - prev;
  if (d >  M_PI) current -= 2.0*M_PI;
  if (d < -M_PI) current += 2.0*M_PI;
  return current;
}

struct Metrics {
  double rms_pos=0.0;
  double rms_th=0.0;
};

static Metrics simulate(const Params& p_true, const Params& p_nom, double T=35.0, double dt=0.01) {
  int N = static_cast<int>(T/dt) + 1;
  PIController ctrl(p_nom);

  // Baseline (nominal plant) for comparison
  std::vector<State> x0(N);
  {
    State x;
    double th_prev = 0.0;
    for (int i=0; i<N; ++i) {
      double t = i*dt;
      double vref, omref; reference(t, vref, omref);
      double tauL, tauR; ctrl.torques(x, vref, omref, dt, tauL, tauR);
      x = rk4_step(x, tauL, tauR, dt, p_nom);
      x.th = wrap_unwrap(th_prev, x.th);
      th_prev = x.th;
      x0[i] = x;
    }
  }

  ctrl.reset();

  // True plant run
  std::vector<double> pos_err(N), th_err(N);
  State x;
  double th_prev = 0.0;
  for (int i=0; i<N; ++i) {
    double t = i*dt;
    double vref, omref; reference(t, vref, omref);
    double tauL, tauR; ctrl.torques(x, vref, omref, dt, tauL, tauR);
    x = rk4_step(x, tauL, tauR, dt, p_true);
    x.th = wrap_unwrap(th_prev, x.th);
    th_prev = x.th;

    double dx = x.px - x0[i].px;
    double dy = x.py - x0[i].py;
    pos_err[i] = std::sqrt(dx*dx + dy*dy);
    th_err[i]  = x.th - x0[i].th;
  }

  auto rms = [](const std::vector<double>& v) {
    double s=0.0;
    for (double x: v) s += x*x;
    return std::sqrt(s / static_cast<double>(v.size()));
  };

  Metrics m;
  m.rms_pos = rms(pos_err);
  m.rms_th  = rms(th_err);
  return m;
}

int main() {
  Params p_nom;

  struct SweepItem {
    std::string name;
    std::vector<double> values;
    std::string unit;
  };

  std::vector<SweepItem> sweep = {
    {"mass_m",      {15.0, 25.0, 40.0}, "kg"},
    {"yaw_Iz",      {1.2,  2.0,  3.5},  "kg*m^2"},
    {"wheel_r",     {0.095,0.10, 0.105},"m"},
    {"wheelbase_b", {0.40, 0.45, 0.52}, "m"},
    {"friction_mu", {0.5,  0.8,  1.0},  "-"},
  };

  std::cout << "Parameter sweep (controller uses nominal params; plant varies)\n";
  for (const auto& item : sweep) {
    for (double v : item.values) {
      Params p_true = p_nom;
      if (item.name == "mass_m") p_true.m  = v;
      if (item.name == "yaw_Iz") p_true.Iz = v;
      if (item.name == "wheel_r") p_true.r = v;
      if (item.name == "wheelbase_b") p_true.b = v;
      if (item.name == "friction_mu") p_true.mu = v;

      Metrics m = simulate(p_true, p_nom);
      std::cout << item.name << "=" << v << " " << item.unit
                << " | pos_rms=" << m.rms_pos << " m"
                << ", theta_rms=" << m.rms_th << " rad\n";
    }
  }
  return 0;
}
