// Chapter12_Lesson1.cpp
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 12 — SLAM II: Graph-Based SLAM
Lesson 1 — Pose Graphs and Factor Graphs

Minimal 2D pose graph Gauss–Newton optimizer (SE(2)) using Eigen.

Build (Linux/macOS):
  g++ -O2 -std=c++17 Chapter12_Lesson1.cpp -I /usr/include/eigen3 -o pose_graph

Run:
  ./pose_graph

Note: This is dense and small-scale by design for teaching.
*/

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

struct Pose2 {
  double x{0}, y{0}, th{0};
};

static double WrapAngle(double a) {
  a = std::fmod(a + M_PI, 2.0*M_PI);
  if (a &lt; 0) a += 2.0*M_PI;
  return a - M_PI;
}

static Eigen::Matrix2d Rot(double th) {
  double c = std::cos(th), s = std::sin(th);
  Eigen::Matrix2d R;
  R &lt;&lt; c, -s,
       s,  c;
  return R;
}

// Local update: t &larr; t + R(th)*d, th &larr; th + dth
static Pose2 BoxPlus(const Pose2&amp; p, const Eigen::Vector3d&amp; d) {
  Pose2 out = p;
  Eigen::Vector2d t(p.x, p.y);
  Eigen::Vector2d inc = Rot(p.th) * d.head&lt;2&gt;();
  t += inc;
  out.x = t.x();
  out.y = t.y();
  out.th = WrapAngle(p.th + d(2));
  return out;
}

static Eigen::Vector3d Between(const Pose2&amp; i, const Pose2&amp; j) {
  Eigen::Vector2d ti(i.x, i.y), tj(j.x, j.y);
  Eigen::Vector2d dt = tj - ti;
  Eigen::Vector2d dxy = Rot(i.th).transpose() * dt;
  double dth = WrapAngle(j.th - i.th);
  return Eigen::Vector3d(dxy.x(), dxy.y(), dth);
}

struct Edge {
  int i{0}, j{0};
  Eigen::Vector3d z;
  Eigen::Matrix3d Omega;
};

static void ResidualAndJacobians(const Pose2&amp; xi, const Pose2&amp; xj, const Eigen::Vector3d&amp; z,
                                 Eigen::Vector3d&amp; r, Eigen::Matrix3d&amp; Ji, Eigen::Matrix3d&amp; Jj)
{
  Eigen::Vector2d ti(xi.x, xi.y), tj(xj.x, xj.y);
  Eigen::Vector2d dt = tj - ti;
  Eigen::Matrix2d RiT = Rot(xi.th).transpose();

  Eigen::Vector3d zhat = Between(xi, xj);
  r = z - zhat;
  r(2) = WrapAngle(r(2));

  // J = [[0,-1],[1,0]]
  Eigen::Matrix2d J;
  J &lt;&lt; 0, -1,
       1,  0;

  Ji.setZero();
  Jj.setZero();

  // dr/dt_i = +RiT
  Ji.block&lt;2,2&gt;(0,0) = RiT;

  // dr/dtheta_i = RiT * J * dt
  Eigen::Vector2d dri_dth = (RiT * (J * dt));
  Ji(0,2) = dri_dth.x();
  Ji(1,2) = dri_dth.y();

  // dr_theta/dtheta_i = +1
  Ji(2,2) = 1.0;

  // dr/dt_j = -RiT
  Jj.block&lt;2,2&gt;(0,0) = -RiT;

  // dr_theta/dtheta_j = -1
  Jj(2,2) = -1.0;
}

static void BuildNormalEquations(const std::vector&lt;Pose2&gt;&amp; poses,
                                 const std::vector&lt;Edge&gt;&amp; edges,
                                 const std::vector&lt;Edge&gt;&amp; priors,
                                 Eigen::MatrixXd&amp; H, Eigen::VectorXd&amp; g)
{
  const int N = static_cast&lt;int&gt;(poses.size());
  const int dim = 3*N;
  H.setZero(dim, dim);
  g.setZero(dim);

  auto sl = [](int idx) { return Eigen::seqN(3*idx, 3); };

  // Relative edges
  for (const auto&amp; e : edges) {
    Eigen::Vector3d r;
    Eigen::Matrix3d Ji, Jj;
    ResidualAndJacobians(poses[e.i], poses[e.j], e.z, r, Ji, Jj);

    Eigen::Matrix3d Hi = Ji.transpose()*e.Omega*Ji;
    Eigen::Matrix3d Hj = Jj.transpose()*e.Omega*Jj;
    Eigen::Matrix3d Hij = Ji.transpose()*e.Omega*Jj;

    Eigen::Vector3d gi = Ji.transpose()*e.Omega*r;
    Eigen::Vector3d gj = Jj.transpose()*e.Omega*r;

    H(sl(e.i), sl(e.i)) += Hi;
    H(sl(e.j), sl(e.j)) += Hj;
    H(sl(e.i), sl(e.j)) += Hij;
    H(sl(e.j), sl(e.i)) += Hij.transpose();

    g(sl(e.i)) += gi;
    g(sl(e.j)) += gj;
  }

  // Priors as unary factors stored in Edge (i==j ignored, uses z as "mu")
  for (const auto&amp; p : priors) {
    int i = p.i;
    Eigen::Vector3d mu = p.z;
    Eigen::Vector3d xi(poses[i].x, poses[i].y, poses[i].th);
    Eigen::Vector3d r = mu - xi;
    r(2) = WrapAngle(r(2));

    Eigen::Matrix3d J = -Eigen::Matrix3d::Identity();
    H(sl(i), sl(i)) += J.transpose()*p.Omega*J;
    g(sl(i)) += J.transpose()*p.Omega*r;
  }
}

static std::vector&lt;Pose2&gt; GaussNewton(std::vector&lt;Pose2&gt; poses,
                                     const std::vector&lt;Edge&gt;&amp; edges,
                                     const std::vector&lt;Edge&gt;&amp; priors,
                                     int iters = 15, double damping = 1e-8)
{
  const int N = static_cast&lt;int&gt;(poses.size());
  for (int k=0; k&lt;iters; ++k) {
    Eigen::MatrixXd H(3*N, 3*N);
    Eigen::VectorXd g(3*N);
    BuildNormalEquations(poses, edges, priors, H, g);
    H += damping * Eigen::MatrixXd::Identity(3*N, 3*N);

    Eigen::VectorXd dx = H.fullPivLu().solve(-g);

    double maxStep = 0.0;
    for (int i=0; i&lt;N; ++i) {
      Eigen::Vector3d d = dx.segment&lt;3&gt;(3*i);
      poses[i] = BoxPlus(poses[i], d);
      maxStep = std::max(maxStep, d.norm());
    }
    std::cout &lt;&lt; "iter " &lt;&lt; k &lt;&lt; ": max|delta|=" &lt;&lt; maxStep &lt;&lt; "\n";
    if (maxStep &lt; 1e-9) break;
  }
  return poses;
}

int main() {
  // A tiny looped trajectory
  std::vector&lt;Pose2&gt; gt = {
    {0,0,0},
    {1,0,0},
    {2,0,0},
    {2,1,M_PI/2},
    {2,2,M_PI/2},
    {1,2,M_PI},
    {0,2,M_PI},
    {0,1,-M_PI/2}
  };
  const int N = static_cast&lt;int&gt;(gt.size());

  std::mt19937 rng(4);
  std::normal_distribution&lt;double&gt; nxy(0.0, 0.02);
  std::normal_distribution&lt;double&gt; nth(0.0, M_PI/180.0);

  Eigen::Matrix3d Omega = Eigen::Matrix3d::Zero();
  Omega(0,0) = 1.0/(0.02*0.02);
  Omega(1,1) = 1.0/(0.02*0.02);
  Omega(2,2) = 1.0/( (M_PI/180.0)*(M_PI/180.0) );

  std::vector&lt;Edge&gt; edges;
  for (int i=0; i&lt;N-1; ++i) {
    Eigen::Vector3d z = Between(gt[i], gt[i+1]);
    z(0) += nxy(rng); z(1) += nxy(rng); z(2) = WrapAngle(z(2) + nth(rng));
    edges.push_back({i, i+1, z, Omega});
  }
  // loop closure 7->0
  {
    Eigen::Vector3d z = Between(gt[7], gt[0]);
    z(0) += nxy(rng); z(1) += nxy(rng); z(2) = WrapAngle(z(2) + nth(rng));
    edges.push_back({7, 0, z, Omega});
  }

  // Initial guess
  std::normal_distribution&lt;double&gt; initxy(0.0, 0.15);
  std::normal_distribution&lt;double&gt; initth(0.0, 5.0*M_PI/180.0);
  std::vector&lt;Pose2&gt; x0 = gt;
  for (auto&amp; p : x0) {
    p.x += initxy(rng);
    p.y += initxy(rng);
    p.th = WrapAngle(p.th + initth(rng));
  }

  // Prior on node 0
  Eigen::Matrix3d Omega0 = Eigen::Matrix3d::Zero();
  Omega0(0,0) = 1e6; Omega0(1,1) = 1e6; Omega0(2,2) = 1e6;
  std::vector&lt;Edge&gt; priors;
  priors.push_back({0, 0, Eigen::Vector3d(gt[0].x, gt[0].y, gt[0].th), Omega0});

  std::cout &lt;&lt; "Optimizing...\n";
  auto est = GaussNewton(x0, edges, priors);

  std::cout &lt;&lt; "\n i | gt(x,y,th)                  | est(x,y,th)\n";
  for (int i=0; i&lt;N; ++i) {
    std::cout &lt;&lt; i &lt;&lt; " | "
              &lt;&lt; gt[i].x &lt;&lt; " " &lt;&lt; gt[i].y &lt;&lt; " " &lt;&lt; gt[i].th &lt;&lt; " | "
              &lt;&lt; est[i].x &lt;&lt; " " &lt;&lt; est[i].y &lt;&lt; " " &lt;&lt; est[i].th &lt;&lt; "\n";
  }
  return 0;
}
