// Chapter2_Lesson5.cpp
/*
Chapter 2 — Wheeled Locomotion Kinematics (Mobile-Specific)
Lesson 5 — Kinematic Calibration of Wheel Parameters

Build (example):
  g++ -O2 -std=c++17 Chapter2_Lesson5.cpp -I/path/to/eigen -o calib_dd

Input CSV format (header optional):
  dphi_L, dphi_R, dx_gt, dy_gt, dtheta_gt
Units:
  dphi_*: radians, dx/dy: meters, dtheta: radians

This program estimates p = [r_L, r_R, b] for a differential-drive robot
via damped Gauss-Newton on relative body-frame increments.
*/

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct DataRow {
  double dphiL, dphiR, dx, dy, dth;
};

static double wrapToPi(double a) {
  const double pi = 3.14159265358979323846;
  a = std::fmod(a + pi, 2.0 * pi);
  if (a < 0) a += 2.0 * pi;
  return a - pi;
}

static double sinc1(double x) {
  if (std::abs(x) < 1e-6) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + (x2 * x2) / 120.0;
  }
  return std::sin(x) / x;
}

static double cosc1(double x) {
  if (std::abs(x) < 1e-6) {
    const double x2 = x * x;
    return x / 2.0 - (x * x2) / 24.0 + (x * x2 * x2) / 720.0;
  }
  return (1.0 - std::cos(x)) / x;
}

static double dsinc1(double x) {
  if (std::abs(x) < 1e-5) {
    // series: -(x)/3 + x^3/30 - x^5/840
    const double x2 = x * x;
    return -(x) / 3.0 + (x * x2) / 30.0 - (x * x2 * x2) / 840.0;
  }
  return (x * std::cos(x) - std::sin(x)) / (x * x);
}

static double dcosc1(double x) {
  if (std::abs(x) < 1e-5) {
    // series: 1/2 - x^2/8 + x^4/144
    const double x2 = x * x;
    return 0.5 - x2 / 8.0 + (x2 * x2) / 144.0;
  }
  return (x * std::sin(x) - (1.0 - std::cos(x))) / (x * x);
}

static bool loadCSV(const std::string& path, std::vector<DataRow>& out) {
  std::ifstream fin(path);
  if (!fin) return false;

  std::string line;
  while (std::getline(fin, line)) {
    if (line.empty()) continue;
    if (line.find("dphi") != std::string::npos) continue;

    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      if (!tok.empty()) vals.push_back(std::stod(tok));
    }
    if (vals.size() < 5) continue;
    out.push_back({vals[0], vals[1], vals[2], vals[3], vals[4]});
  }
  return true;
}

static void residualAndJacobian(const std::vector<DataRow>& data,
                                const Eigen::Vector3d& p,
                                Eigen::VectorXd& r,
                                Eigen::MatrixXd& J) {
  const double rL = p(0), rR = p(1), b = p(2);
  const int N = static_cast<int>(data.size());
  r.resize(3 * N);
  J.resize(3 * N, 3);

  for (int k = 0; k < N; ++k) {
    const auto& d = data[k];
    const double sL = rL * d.dphiL;
    const double sR = rR * d.dphiR;
    const double A = sR + sL;
    const double B = sR - sL;

    const double ds = 0.5 * A;
    const double dth = B / b;

    const double f = sinc1(dth);
    const double g = cosc1(dth);
    const double fp = dsinc1(dth);
    const double gp = dcosc1(dth);

    const double dx = ds * f;
    const double dy = ds * g;

    const double ex = dx - d.dx;
    const double ey = dy - d.dy;
    const double eth = wrapToPi(dth - d.dth);

    r(3 * k + 0) = ex;
    r(3 * k + 1) = ey;
    r(3 * k + 2) = eth;

    // derivatives
    const double dds_drL = 0.5 * d.dphiL;
    const double dds_drR = 0.5 * d.dphiR;

    const double ddth_drL = -(d.dphiL) / b;
    const double ddth_drR = (d.dphiR) / b;
    const double ddth_db  = -(B) / (b * b);

    const double ddx_drL = dds_drL * f + ds * fp * ddth_drL;
    const double ddx_drR = dds_drR * f + ds * fp * ddth_drR;
    const double ddx_db  = 0.0      * f + ds * fp * ddth_db;

    const double ddy_drL = dds_drL * g + ds * gp * ddth_drL;
    const double ddy_drR = dds_drR * g + ds * gp * ddth_drR;
    const double ddy_db  = 0.0      * g + ds * gp * ddth_db;

    J(3 * k + 0, 0) = ddx_drL;
    J(3 * k + 0, 1) = ddx_drR;
    J(3 * k + 0, 2) = ddx_db;

    J(3 * k + 1, 0) = ddy_drL;
    J(3 * k + 1, 1) = ddy_drR;
    J(3 * k + 1, 2) = ddy_db;

    J(3 * k + 2, 0) = ddth_drL;
    J(3 * k + 2, 1) = ddth_drR;
    J(3 * k + 2, 2) = ddth_db;
  }
}

static Eigen::Vector3d calibrateGN(const std::vector<DataRow>& data,
                                  Eigen::Vector3d p0,
                                  int maxIters = 60) {
  Eigen::Vector3d p = p0;
  double lambda = 1e-3;

  for (int it = 0; it < maxIters; ++it) {
    Eigen::VectorXd r;
    Eigen::MatrixXd J;
    residualAndJacobian(data, p, r, J);

    const Eigen::Matrix3d H = J.transpose() * J;
    const Eigen::Vector3d g = J.transpose() * r;

    Eigen::Matrix3d A = H + lambda * Eigen::Matrix3d::Identity();
    Eigen::Vector3d dp = -A.ldlt().solve(g);

    if (dp.norm() < 1e-12) break;

    Eigen::Vector3d pNew = p + dp;
    Eigen::VectorXd rNew;
    Eigen::MatrixXd JNew;
    residualAndJacobian(data, pNew, rNew, JNew);

    if (rNew.squaredNorm() < r.squaredNorm()) {
      p = pNew;
      lambda *= 0.7;
    } else {
      lambda *= 2.0;
    }
  }
  return p;
}

int main(int argc, char** argv) {
  std::string csvPath = "";
  if (argc >= 2) csvPath = argv[1];

  std::vector<DataRow> data;
  if (!csvPath.empty()) {
    if (!loadCSV(csvPath, data)) {
      std::cerr << "Failed to read CSV: " << csvPath << "\n";
      return 1;
    }
  } else {
    std::cerr << "Usage: calib_dd path/to/data.csv\n";
    return 1;
  }

  Eigen::Vector3d p0(0.05, 0.05, 0.30);
  Eigen::Vector3d phat = calibrateGN(data, p0);

  std::cout << "Estimated [rL, rR, b] = " << phat.transpose() << "\n";

  // RMS residuals
  Eigen::VectorXd r;
  Eigen::MatrixXd J;
  residualAndJacobian(data, phat, r, J);
  double rms_dx = 0, rms_dy = 0, rms_th = 0;
  const int N = static_cast<int>(data.size());
  for (int k = 0; k < N; ++k) {
    rms_dx += r(3*k+0) * r(3*k+0);
    rms_dy += r(3*k+1) * r(3*k+1);
    rms_th += r(3*k+2) * r(3*k+2);
  }
  rms_dx = std::sqrt(rms_dx / N);
  rms_dy = std::sqrt(rms_dy / N);
  rms_th = std::sqrt(rms_th / N);
  std::cout << "RMS [dx, dy, dtheta] = " << rms_dx << ", " << rms_dy << ", " << rms_th << "\n";
  return 0;
}
