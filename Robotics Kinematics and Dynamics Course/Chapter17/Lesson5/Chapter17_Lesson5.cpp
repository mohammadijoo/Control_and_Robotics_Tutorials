#include <iostream>
#include <Eigen/Dense>

struct LegParams {
  double m_b, m1, m2;
  double I_b, I1, I2;
  double l1, l2, c1, c2;
  double g;
};

using Vec5 = Eigen::Matrix<double, 5, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Mat5 = Eigen::Matrix<double, 5, 5>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat25 = Eigen::Matrix<double, 2, 5>;
using Mat52 = Eigen::Matrix<double, 5, 2>;

// These functions should be implemented from the planar model or via a library.
Mat5 massMatrix(const Vec5& q, const LegParams& p);
Vec5 biasTerm(const Vec5& q, const Vec5& dq, const LegParams& p); // h(q, dq)
Mat25 contactJacobian(const Vec5& q, const LegParams& p);
Mat25 contactJdotV(const Vec5& q, const Vec5& dq, const LegParams& p); // Jdot_c dq

struct DynamicsResult {
  Vec5 ddq;
  Vec2 lambda;
};

DynamicsResult constrainedDynamics(const Vec5& q,
                                   const Vec5& dq,
                                   const Vec2& tau,
                                   const LegParams& p)
{
  Mat5 M = massMatrix(q, p);
  Vec5 h = biasTerm(q, dq, p);
  Mat25 Jc = contactJacobian(q, p);
  Mat25 Jc_dot_v = contactJdotV(q, dq, p);

  // Selection matrix S (2 x 5)
  Eigen::Matrix<double, 2, 5> S;
  S.setZero();
  S(0, 3) = 1.0; // hip
  S(1, 4) = 1.0; // knee

  // Assemble block system
  Mat2 Z = Mat2::Zero();
  Eigen::Matrix<double, 7, 7> A;
  A.setZero();
  // Top-left: M
  A.block<5,5>(0, 0) = M;
  // Top-right: -Jc^T
  A.block<5,2>(0, 5) = -Jc.transpose();
  // Bottom-left: Jc
  A.block<2,5>(5, 0) = Jc;
  // Bottom-right: zeros

  Eigen::Matrix<double, 7, 1> rhs;
  rhs.setZero();
  rhs.segment<5>(0) = S.transpose() * tau - h;
  rhs.segment<2>(5) = -Jc_dot_v * dq;  // Jdot_c dq

  Eigen::Matrix<double, 7, 1> sol = A.fullPivLu().solve(rhs);

  DynamicsResult res;
  res.ddq = sol.segment<5>(0);
  res.lambda = sol.segment<2>(5);
  return res;
}

int main() {
  LegParams p{/* m_b */ 10.0, /* m1 */ 3.0, /* m2 */ 2.0,
              /* I_b */ 1.0,  /* I1 */ 0.2, /* I2 */ 0.1,
              /* l1 */ 0.5,   /* l2 */ 0.5,
              /* c1 */ 0.25,  /* c2 */ 0.25,
              /* g  */ 9.81};

  Vec5 q;   q << 0.0, 0.9, 0.0, 0.4, -0.8;
  Vec5 dq;  dq.setZero();
  Vec2 tau; tau << 0.0, 0.0;

  DynamicsResult res = constrainedDynamics(q, dq, tau, p);
  std::cout << "ddq = " << res.ddq.transpose() << std::endl;
  std::cout << "lambda = " << res.lambda.transpose() << std::endl;
  return 0;
}
      
