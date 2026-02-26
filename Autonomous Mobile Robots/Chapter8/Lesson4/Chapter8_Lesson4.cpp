// Chapter8_Lesson4.cpp
// Particle-Filter Localization — Degeneracy + Kidnapped Robot Recovery (2D)
// Build (example):
//   g++ -O2 -std=c++17 -o Chapter8_Lesson4 Chapter8_Lesson4.cpp
//
// Output:
//   Chapter8_Lesson4_results.csv

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

struct Pose {
  double x{0}, y{0}, th{0};
};

struct Particle {
  double x{0}, y{0}, th{0}, w{0};
};

static double wrap_angle(double a) {
  const double pi = 3.14159265358979323846;
  a = std::fmod(a + pi, 2.0 * pi);
  if (a < 0) a += 2.0 * pi;
  return a - pi;
}

static Particle motion_step(const Particle& p, double v, double w, double dt,
                            double sigma_v, double sigma_w,
                            std::mt19937_64& gen) {
  std::normal_distribution<double> nv(0.0, sigma_v);
  std::normal_distribution<double> nw(0.0, sigma_w);

  double v_n = v + nv(gen);
  double w_n = w + nw(gen);

  Particle q = p;
  if (std::abs(w_n) < 1e-9) {
    q.x = p.x + v_n * dt * std::cos(p.th);
    q.y = p.y + v_n * dt * std::sin(p.th);
    q.th = p.th;
  } else {
    q.x = p.x + (v_n / w_n) * (std::sin(p.th + w_n * dt) - std::sin(p.th));
    q.y = p.y - (v_n / w_n) * (std::cos(p.th + w_n * dt) - std::cos(p.th));
    q.th = p.th + w_n * dt;
  }
  q.th = wrap_angle(q.th);
  return q;
}

static void measurement_model(const Pose& x,
                              const std::vector<std::pair<double,double>>& landmarks,
                              std::vector<std::pair<double,double>>& out_rb) {
  out_rb.resize(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); ++i) {
    double dx = landmarks[i].first - x.x;
    double dy = landmarks[i].second - x.y;
    double r = std::sqrt(dx*dx + dy*dy);
    double b = std::atan2(dy, dx) - x.th;
    b = wrap_angle(b);
    out_rb[i] = {r, b};
  }
}

static double log_gauss_0(double e, double sigma) {
  const double pi = 3.14159265358979323846;
  return -0.5 * (e/sigma)*(e/sigma) - std::log(std::sqrt(2.0*pi)*sigma);
}

static double log_sensor_likelihood(const Particle& p,
                                    const std::vector<std::pair<double,double>>& z,
                                    const std::vector<std::pair<double,double>>& landmarks,
                                    double sigma_r, double sigma_b) {
  std::vector<std::pair<double,double>> zh;
  measurement_model(Pose{p.x, p.y, p.th}, landmarks, zh);

  double ll = 0.0;
  for (size_t i = 0; i < z.size(); ++i) {
    double dr = z[i].first  - zh[i].first;
    double db = wrap_angle(z[i].second - zh[i].second);
    ll += log_gauss_0(dr, sigma_r);
    ll += log_gauss_0(db, sigma_b);
  }
  return ll;
}

static void normalize(std::vector<Particle>& P) {
  double s = 0.0;
  for (const auto& p : P) s += p.w;
  if (!(s > 0.0) || !std::isfinite(s)) {
    double w = 1.0 / static_cast<double>(P.size());
    for (auto& p : P) p.w = w;
    return;
  }
  for (auto& p : P) p.w /= s;
}

static double Neff(const std::vector<Particle>& P) {
  double s2 = 0.0;
  for (const auto& p : P) s2 += p.w * p.w;
  if (s2 <= 0.0) return 0.0;
  return 1.0 / s2;
}

static std::vector<Particle> systematic_resample(const std::vector<Particle>& P,
                                                 std::mt19937_64& gen) {
  const size_t N = P.size();
  std::vector<double> cdf(N);
  double acc = 0.0;
  for (size_t i = 0; i < N; ++i) {
    acc += P[i].w;
    cdf[i] = acc;
  }
  cdf[N-1] = 1.0;

  std::uniform_real_distribution<double> ur(0.0, 1.0);
  double u0 = ur(gen) / static_cast<double>(N);

  std::vector<Particle> out;
  out.reserve(N);

  size_t i = 0;
  for (size_t m = 0; m < N; ++m) {
    double u = u0 + static_cast<double>(m) / static_cast<double>(N);
    while (u > cdf[i]) ++i;
    Particle q = P[i];
    q.w = 1.0 / static_cast<double>(N);
    out.push_back(q);
  }
  return out;
}

static void roughen(std::vector<Particle>& P, double k,
                    double xmin, double xmax, double ymin, double ymax,
                    std::mt19937_64& gen) {
  const double pi = 3.14159265358979323846;
  const double N = static_cast<double>(P.size());
  const double d = 3.0;
  double sx = k * (xmax - xmin) * std::pow(N, -1.0/d);
  double sy = k * (ymax - ymin) * std::pow(N, -1.0/d);
  double sth = k * (2.0*pi) * std::pow(N, -1.0/d);

  std::normal_distribution<double> nx(0.0, sx);
  std::normal_distribution<double> ny(0.0, sy);
  std::normal_distribution<double> nth(0.0, sth);

  for (auto& p : P) {
    p.x += nx(gen);
    p.y += ny(gen);
    p.th = wrap_angle(p.th + nth(gen));
  }
}

static void inject_random(std::vector<Particle>& P, double frac,
                          double xmin, double xmax, double ymin, double ymax,
                          std::mt19937_64& gen) {
  const size_t N = P.size();
  size_t m = static_cast<size_t>(std::llround(std::max(0.0, std::min(1.0, frac)) * static_cast<double>(N)));
  if (m == 0) return;

  std::uniform_real_distribution<double> ux(xmin, xmax);
  std::uniform_real_distribution<double> uy(ymin, ymax);
  std::uniform_real_distribution<double> uth(-3.14159265358979323846, 3.14159265358979323846);

  for (size_t i = 0; i < m; ++i) {
    P[i].x = ux(gen);
    P[i].y = uy(gen);
    P[i].th = uth(gen);
  }
}

static Pose estimate(const std::vector<Particle>& P) {
  double x=0.0, y=0.0;
  double c=0.0, s=0.0;
  for (const auto& p : P) {
    x += p.w * p.x;
    y += p.w * p.y;
    c += p.w * std::cos(p.th);
    s += p.w * std::sin(p.th);
  }
  return Pose{x, y, std::atan2(s, c)};
}

int main() {
  std::mt19937_64 gen(4);

  double xmin=0.0, xmax=10.0, ymin=0.0, ymax=10.0;
  std::vector<std::pair<double,double>> landmarks{
    {2.0,2.0},{8.0,2.0},{8.0,8.0},{2.0,8.0}
  };

  double dt = 0.1;
  int T = 300;
  int kidnapped_t = 170;

  double sigma_v = 0.05, sigma_w = 0.03;
  double sigma_r = 0.15, sigma_b = 0.07;

  int N = 800;
  double Neff_ratio = 0.5;
  double rough_k = 0.15;

  double eps_min = 0.01, eps_max = 0.30;
  double ll_thresh = -12.0;

  Pose xtrue{1.0, 1.0, 0.0};

  std::uniform_real_distribution<double> ux(xmin, xmax);
  std::uniform_real_distribution<double> uy(ymin, ymax);
  std::uniform_real_distribution<double> uth(-3.14159265358979323846, 3.14159265358979323846);

  std::vector<Particle> P;
  P.reserve(N);
  for (int i = 0; i < N; ++i) {
    P.push_back(Particle{ux(gen), uy(gen), uth(gen), 1.0/static_cast<double>(N)});
  }

  auto control = [](int t) {
    if (t < 70)  return std::pair<double,double>{0.7, 0.0};
    if (t < 90)  return std::pair<double,double>{0.7, 0.9};
    if (t < 160) return std::pair<double,double>{0.7, 0.0};
    if (t < 180) return std::pair<double,double>{0.7, 0.9};
    if (t < 250) return std::pair<double,double>{0.7, 0.0};
    return std::pair<double,double>{0.7, 0.9};
  };

  std::normal_distribution<double> nr(0.0, sigma_r);
  std::normal_distribution<double> nb(0.0, sigma_b);

  std::ofstream f("Chapter8_Lesson4_results.csv");
  f << "t,true_x,true_y,true_th,est_x,est_y,est_th,Neff,eps\n";

  std::vector<std::pair<double,double>> zhat, z;

  for (int t = 0; t < T; ++t) {
    auto [v_cmd, w_cmd] = control(t);

    Particle tmp{xtrue.x, xtrue.y, xtrue.th, 1.0};
    tmp = motion_step(tmp, v_cmd, w_cmd, dt, sigma_v, sigma_w, gen);
    xtrue = Pose{tmp.x, tmp.y, tmp.th};

    if (t == kidnapped_t) {
      xtrue = Pose{ux(gen), uy(gen), uth(gen)};
    }

    measurement_model(xtrue, landmarks, zhat);
    z = zhat;
    for (auto& rb : z) {
      rb.first  += nr(gen);
      rb.second = wrap_angle(rb.second + nb(gen));
    }

    for (auto& p : P) {
      Particle q = motion_step(p, v_cmd, w_cmd, dt, sigma_v, sigma_w, gen);
      p.x = q.x; p.y = q.y; p.th = q.th;
    }

    std::vector<double> lls(N);
    double llmax = -1e300;
    for (int i = 0; i < N; ++i) {
      lls[i] = log_sensor_likelihood(P[i], z, landmarks, sigma_r, sigma_b);
      llmax = std::max(llmax, lls[i]);
    }
    for (int i = 0; i < N; ++i) {
      P[i].w = std::exp(lls[i] - llmax);
    }
    normalize(P);

    double lbar = 0.0;
    for (int i = 0; i < N; ++i) {
      lbar += P[i].w * log_sensor_likelihood(P[i], z, landmarks, sigma_r, sigma_b);
    }
    double eps = (lbar < ll_thresh) ? eps_max : eps_min;

    double neff = Neff(P);
    if (neff < Neff_ratio * static_cast<double>(N)) {
      P = systematic_resample(P, gen);
      roughen(P, rough_k, xmin, xmax, ymin, ymax, gen);
      inject_random(P, eps, xmin, xmax, ymin, ymax, gen);

      llmax = -1e300;
      for (int i = 0; i < N; ++i) {
        lls[i] = log_sensor_likelihood(P[i], z, landmarks, sigma_r, sigma_b);
        llmax = std::max(llmax, lls[i]);
      }
      for (int i = 0; i < N; ++i) {
        P[i].w = std::exp(lls[i] - llmax);
      }
      normalize(P);
    }

    Pose xest = estimate(P);

    f << t << "," << xtrue.x << "," << xtrue.y << "," << xtrue.th << ","
      << xest.x << "," << xest.y << "," << xest.th << ","
      << neff << "," << eps << "\n";
  }

  std::cout << "Saved: Chapter8_Lesson4_results.csv\n";
  return 0;
}
