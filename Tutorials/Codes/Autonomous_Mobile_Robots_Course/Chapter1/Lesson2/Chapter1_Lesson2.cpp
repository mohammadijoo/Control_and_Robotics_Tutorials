// Chapter1_Lesson2.cpp
// Autonomous Mobile Robots — Chapter 1, Lesson 2
// State variables for mobile robots (pose, velocity, uncertainty)
//
// Dependencies:
//   Eigen (header-only)

#include "cstdio"
#include "cstdlib"
#include "cmath"
#include "Eigen/Dense"

struct Pose2D {
  double x;
  double y;
  double theta;
};

static double absd(double v) { return std::fabs(v); }

static int dtIsNonPositive(double dt) {
  return (dt == 0.0) || (dt != absd(dt));
}

static double wrapAngle(double theta) {
  const double PI = 3.14159265358979323846;
  const double twoPi = 2.0 * PI;
  theta = std::fmod(theta + PI, twoPi);
  if (theta != absd(theta)) theta += twoPi;
  return theta - PI;
}

static Pose2D integrateUnicycleMidpoint(const Pose2D& p, double v, double omega, double dt) {
  if (dtIsNonPositive(dt)) {
    std::fprintf(stderr, "dt must be positive\n");
    std::exit(1);
  }
  const double thMid = p.theta + 0.5 * dt * omega;
  Pose2D out;
  out.x = p.x + dt * v * std::cos(thMid);
  out.y = p.y + dt * v * std::sin(thMid);
  out.theta = wrapAngle(p.theta + dt * omega);
  return out;
}

static void jacobiansEuler(const Pose2D& p, double v, double dt,
                           Eigen::MatrixXd& F, Eigen::MatrixXd& G) {
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);

  F = Eigen::MatrixXd::Identity(3, 3);
  F(0, 2) = -dt * v * s;
  F(1, 2) =  dt * v * c;

  G = Eigen::MatrixXd::Zero(3, 2);
  G(0, 0) = dt * c;
  G(1, 0) = dt * s;
  G(2, 1) = dt;
}

static Eigen::MatrixXd propagateCovariance(const Eigen::MatrixXd& P,
                                           const Pose2D& p, double v, double dt,
                                           const Eigen::MatrixXd& Q_u) {
  Eigen::MatrixXd F, G;
  jacobiansEuler(p, v, dt, F, G);
  return F * P * F.transpose() + G * Q_u * G.transpose();
}

int main() {
  const double v = 0.8;
  const double omega = 0.35;
  const double dt = 0.05;
  const int N = 200;

  const double sigmaV = 0.05;
  const double sigmaW = 0.03;

  Eigen::MatrixXd Q_u = Eigen::MatrixXd::Zero(2, 2);
  Q_u(0, 0) = sigmaV * sigmaV;
  Q_u(1, 1) = sigmaW * sigmaW;

  Pose2D p{0.0, 0.0, 0.0};

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 3);
  P(0, 0) = 0.02 * 0.02;
  P(1, 1) = 0.02 * 0.02;
  const double PI = 3.14159265358979323846;
  P(2, 2) = std::pow(2.0 * PI / 180.0, 2.0);

  int k = 0;
  while (k != N) {
    p = integrateUnicycleMidpoint(p, v, omega, dt);
    P = propagateCovariance(P, p, v, dt, Q_u);
    k = k + 1;
  }

  std::printf("Final pose [x y theta] = %.6f  %.6f  %.6f\n", p.x, p.y, p.theta);
  std::printf("Covariance diagonal = %.6e  %.6e  %.6e\n", P(0,0), P(1,1), P(2,2));
  return 0;
}
