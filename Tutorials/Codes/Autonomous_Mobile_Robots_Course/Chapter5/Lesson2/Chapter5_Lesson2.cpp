/*
Chapter 5 — Odometry and Dead Reckoning
Lesson 2: IMU Integration for Ground Robots

Filename: Chapter5_Lesson2.cpp

Build (example):
  g++ -O2 -std=c++17 Chapter5_Lesson2.cpp -o Chapter5_Lesson2

This program implements:
- Quaternion strapdown attitude propagation
- Acceleration integration for velocity/position in navigation frame
- Optional planar constraints (z=0, roll=pitch=0 by yaw-only projection)

Input: (optional) a CSV "imu.csv" with columns:
  t, gx, gy, gz, ax, ay, az
If not provided, it runs an internal synthetic demo.
*/

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Vec3 {
  double x{0}, y{0}, z{0};
  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return Vec3{x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return Vec3{x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return Vec3{x * s, y * s, z * s}; }
};

static inline double dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline double norm(const Vec3& a) { return std::sqrt(dot(a,a)); }

struct Mat3 {
  double m[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
  Vec3 mul(const Vec3& v) const {
    return Vec3{
      m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
      m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
      m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
    };
  }
};

struct Quat { // w, x, y, z
  double w{1}, x{0}, y{0}, z{0};

  void normalize() {
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n <= 0) { w=1; x=y=z=0; return; }
    w/=n; x/=n; y/=n; z/=n;
  }

  static Quat mul(const Quat& a, const Quat& b) {
    return Quat{
      a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
  }

  static Quat fromDeltaTheta(const Vec3& dtheta) {
    double angle = norm(dtheta);
    if (angle < 1e-12) {
      return Quat{1.0, 0.5*dtheta.x, 0.5*dtheta.y, 0.5*dtheta.z};
    }
    Vec3 axis{dtheta.x/angle, dtheta.y/angle, dtheta.z/angle};
    double half = 0.5*angle;
    double s = std::sin(half);
    return Quat{std::cos(half), axis.x*s, axis.y*s, axis.z*s};
  }

  Mat3 toR() const {
    Mat3 R;
    const double ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    R.m[0][0] = 1 - 2*(yy + zz);
    R.m[0][1] = 2*(x*y - w*z);
    R.m[0][2] = 2*(x*z + w*y);

    R.m[1][0] = 2*(x*y + w*z);
    R.m[1][1] = 1 - 2*(xx + zz);
    R.m[1][2] = 2*(y*z - w*x);

    R.m[2][0] = 2*(x*z - w*y);
    R.m[2][1] = 2*(y*z + w*x);
    R.m[2][2] = 1 - 2*(xx + yy);
    return R;
  }

  double yaw() const {
    // yaw = atan2(2(w z + x y), 1 - 2(y^2 + z^2))
    return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
  }

  static Quat fromYaw(double yaw) {
    double half = 0.5*yaw;
    return Quat{std::cos(half), 0.0, 0.0, std::sin(half)};
  }
};

static inline double wrapPi(double a) {
  a = std::fmod(a + M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  return a - M_PI;
}

struct Params {
  double g = 9.80665;
  Vec3 bg{0,0,0};
  Vec3 ba{0,0,0};
  bool enforcePlanar = true;
};

struct Sample {
  double t{0};
  Vec3 gyro{0,0,0};  // rad/s
  Vec3 accel{0,0,0}; // m/s^2 specific force
};

static bool loadCsv(const std::string& path, std::vector<Sample>& out) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      try { vals.push_back(std::stod(tok)); }
      catch (...) { vals.clear(); break; }
    }
    if (vals.size() != 7) continue;
    Sample s;
    s.t = vals[0];
    s.gyro = Vec3{vals[1], vals[2], vals[3]};
    s.accel = Vec3{vals[4], vals[5], vals[6]};
    out.push_back(s);
  }
  return !out.empty();
}

static void integrate(const std::vector<Sample>& data, const Params& P) {
  Vec3 p{0,0,0}, v{0,0,0};
  Quat q{};
  const Vec3 gnav{0,0,-P.g};

  for (size_t k = 0; k < data.size(); ++k) {
    double dt = (k==0) ? (data[1].t - data[0].t) : (data[k].t - data[k-1].t);

    // Attitude propagation
    Vec3 omega = data[k].gyro - P.bg;
    Quat dq = Quat::fromDeltaTheta(omega * dt);
    q = Quat::mul(q, dq);
    q.normalize();

    if (P.enforcePlanar) {
      q = Quat::fromYaw(q.yaw());
    }

    // Acceleration to nav frame: a = R f + g
    Vec3 fb = data[k].accel - P.ba;
    Mat3 R = q.toR();
    Vec3 anav = R.mul(fb) + gnav;

    // Integrate (semi-implicit Euler)
    v = v + anav * dt;
    p = p + v * dt + anav * (0.5*dt*dt);

    if (P.enforcePlanar) { p.z = 0; v.z = 0; }

    if ((k % 100) == 0) {
      std::cout << "t=" << data[k].t
                << " p=(" << p.x << "," << p.y << "," << p.z << ")"
                << " yaw[deg]=" << (180.0/M_PI)*q.yaw()
                << "\n";
    }
  }
}

static std::vector<Sample> syntheticDemo() {
  const int N = 2000;
  const double dt = 0.01;
  const double v0 = 1.2;
  const double w0 = 0.20;

  std::vector<Sample> data;
  data.reserve(N);

  for (int k=0; k<N; ++k) {
    double t = k*dt;
    double yaw = w0*t;
    double ax = v0*w0*std::cos(yaw);
    double ay = v0*w0*std::sin(yaw);

    // specific force in body: f_b = R_bn (a - g)
    double c = std::cos(yaw), s = std::sin(yaw);
    // R_bn = R_nb^T = Rz(-yaw)
    double fx =  c*ax + s*ay;  // z=0
    double fy = -s*ax + c*ay;
    double fz = 9.80665;

    Sample smp;
    smp.t = t;
    smp.gyro = Vec3{0,0,w0};
    smp.accel = Vec3{fx, fy, fz};
    data.push_back(smp);
  }
  return data;
}

int main(int argc, char** argv) {
  std::vector<Sample> data;
  if (argc >= 2) {
    if (!loadCsv(argv[1], data)) {
      std::cerr << "Failed to load CSV; running synthetic demo.\n";
      data = syntheticDemo();
    }
  } else {
    data = syntheticDemo();
  }

  Params P;
  P.enforcePlanar = true;
  integrate(data, P);
  return 0;
}
