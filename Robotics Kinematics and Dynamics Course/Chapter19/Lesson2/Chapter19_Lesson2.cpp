#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/regressor.hpp>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct MultiSineParams {
  MatrixXd A;      // (nq x M)
  Eigen::VectorXd W;
  Eigen::VectorXd PHI0;
  VectorXd q0;
};

void multiSineJoint(const Eigen::VectorXd &t,
                    double q0,
                    const Eigen::VectorXd &A_row,
                    const Eigen::VectorXd &W,
                    const Eigen::VectorXd &PHI0,
                    Eigen::VectorXd &q,
                    Eigen::VectorXd &qd,
                    Eigen::VectorXd &qdd)
{
  const int N = static_cast<int>(t.size());
  const int M = static_cast<int>(W.size());
  q  = VectorXd::Constant(N, q0);
  qd = VectorXd::Zero(N);
  qdd = VectorXd::Zero(N);
  for (int m = 0; m < M; ++m) {
    double w = W[m];
    double a = A_row[m];
    double ph = PHI0[m];
    for (int k = 0; k < N; ++k) {
      double arg = w * t[k] + ph;
      double s = std::sin(arg);
      double c = std::cos(arg);
      q[k]  += a * s;
      qd[k] += a * w * c;
      qdd[k] += -a * w * w * s;
    }
  }
}

int main()
{
  // 1) Load model
  pinocchio::Model model;
  pinocchio::urdf::buildModel("path/to/your_robot.urdf", model);
  pinocchio::Data data(model);
  const int nq = static_cast<int>(model.nq);
  const int nv = static_cast<int>(model.nv);

  // 2) Time grid
  double T = 5.0;
  double dt = 0.002;
  int N = static_cast<int>(T / dt);
  VectorXd t(N);
  for (int k = 0; k < N; ++k) t[k] = k * dt;

  // 3) Example parameters
  int M = 3;
  MultiSineParams P;
  P.A = MatrixXd::Constant(nq, M, 0.4);
  P.W = VectorXd::LinSpaced(M, 1.0, 5.0);
  P.PHI0 = VectorXd::LinSpaced(M, 0.0, M_PI / 2.0);
  P.q0 = VectorXd::Zero(nq);

  // 4) Build trajectories
  MatrixXd q_traj(N, nq), qd_traj(N, nq), qdd_traj(N, nq);
  for (int i = 0; i < nq; ++i) {
    Eigen::VectorXd q, qd, qdd;
    multiSineJoint(t, P.q0[i], P.A.row(i).transpose(), P.W, P.PHI0,
                   q, qd, qdd);
    q_traj.col(i) = q;
    qd_traj.col(i) = qd;
    qdd_traj.col(i) = qdd;
  }

  // 5) Stack regressor
  // p is dynamic parameter dimension (query from a first call)
  // Here we create Phi with unknown p: start with first sample
  VectorXd q0_vec = q_traj.row(0).transpose();
  VectorXd qd0_vec = qd_traj.row(0).transpose();
  VectorXd qdd0_vec = qdd_traj.row(0).transpose();
  MatrixXd Y0 = pinocchio::computeJointTorqueRegressor(
      model, data, q0_vec, qd0_vec, qdd0_vec);
  int p = static_cast<int>(Y0.cols());

  MatrixXd Phi(N * nv, p);
  Phi.block(0, 0, nv, p) = Y0;

  for (int k = 1; k < N; ++k) {
    VectorXd qk = q_traj.row(k).transpose();
    VectorXd qdk = qd_traj.row(k).transpose();
    VectorXd qddk = qdd_traj.row(k).transpose();
    MatrixXd Yk = pinocchio::computeJointTorqueRegressor(
        model, data, qk, qdk, qddk);
    Phi.block(k * nv, 0, nv, p) = Yk;
  }

  // 6) Information matrix and condition number
  MatrixXd F = Phi.transpose() * Phi;
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(F);
  double lambda_min = es.eigenvalues().minCoeff();
  double lambda_max = es.eigenvalues().maxCoeff();
  double cond = lambda_max / lambda_min;
  std::cout << "lambda_min = " << lambda_min << std::endl;
  std::cout << "cond(F)    = " << cond << std::endl;

  return 0;
}
      
