#include <iostream>
#include <vector>
#include <Eigen/Dense>
// Include your QP solver headers, e.g. OSQP

struct Obstacle {
  Eigen::Vector2d center;
  double radius;
};

struct TrajOptProblem {
  int N; // number of segments
  double d_min;
  double lambda_smooth;
  double trust_radius;
  Eigen::Vector2d q_start;
  Eigen::Vector2d q_goal;
  std::vector<Obstacle> obstacles;
};

void buildQP(const TrajOptProblem& prob,
             const Eigen::MatrixXd& traj,
             Eigen::MatrixXd& H,
             Eigen::VectorXd& f,
             Eigen::MatrixXd& A,
             Eigen::VectorXd& lb,
             Eigen::VectorXd& ub)
{
  const int N = prob.N;
  const int dim = 2;
  const int nvar = (N + 1) * dim;

  H.setZero(nvar, nvar);
  f.setZero(nvar);

  // Smoothness: sum ||q_{k+1} - 2 q_k + q_{k-1}||^2
  for (int k = 1; k < N; ++k) {
    for (int d = 0; d < dim; ++d) {
      int i_prev = (k - 1) * dim + d;
      int i_curr = k * dim + d;
      int i_next = (k + 1) * dim + d;

      // Contribution of (q_next - 2 q_curr + q_prev)^2
      H(i_prev, i_prev) += prob.lambda_smooth;
      H(i_curr, i_curr) += 4.0 * prob.lambda_smooth;
      H(i_next, i_next) += prob.lambda_smooth;

      H(i_prev, i_curr) += -2.0 * prob.lambda_smooth;
      H(i_curr, i_prev) += -2.0 * prob.lambda_smooth;

      H(i_curr, i_next) += -2.0 * prob.lambda_smooth;
      H(i_next, i_curr) += -2.0 * prob.lambda_smooth;

      H(i_prev, i_next) += 1.0 * prob.lambda_smooth;
      H(i_next, i_prev) += 1.0 * prob.lambda_smooth;
    }
  }

  // Goal term: ||q_N - q_goal||^2
  for (int d = 0; d < dim; ++d) {
    int idx = N * dim + d;
    H(idx, idx) += 1.0;
    f(idx) += -2.0 * prob.q_goal[d];
  }

  // We now assemble linear constraints of the form
  // lb <= A x <= ub
  // including equality constraints for q_0, q_N,
  // trust-region, and linearized collision constraints.

  // For brevity, we omit the detailed construction.
  // In practice, you would stack each linear constraint row-by-row.

  // Example: q_0 == q_start
  // A(row, idx_of_q0_d) = 1; lb(row) = ub(row) = q_start[d];

  // Example: trust region around traj
  // For each component i: -trust_radius <= x_i - traj_i <= trust_radius
  // A(row, i) = 1; lb(row) = traj_i - trust_radius; ub(row) = traj_i + trust_radius;

  // Example: linearized collision constraint
  // d + grad^T (q_k - qk_curr) >= d_min
  // Rewritten as: grad^T q_k >= d_min - d + grad^T qk_curr

  // After constructing H, f, A, lb, ub, call the QP solver.
}

int main() {
  TrajOptProblem prob;
  prob.N = 20;
  prob.d_min = 0.1;
  prob.lambda_smooth = 1.0;
  prob.trust_radius = 0.3;
  prob.q_start = Eigen::Vector2d(0.0, 0.0);
  prob.q_goal  = Eigen::Vector2d(1.0, 1.0);
  prob.obstacles.push_back({Eigen::Vector2d(0.5, 0.5), 0.2});

  // Initial straight-line trajectory
  Eigen::MatrixXd traj(prob.N + 1, 2);
  for (int k = 0; k <= prob.N; ++k) {
    double s = static_cast<double>(k) / prob.N;
    traj.row(k) = (1.0 - s) * prob.q_start + s * prob.q_goal;
  }

  // SCO outer loop (omitting convergence tests for brevity)
  for (int it = 0; it < 10; ++it) {
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::MatrixXd A;
    Eigen::VectorXd lb, ub;

    buildQP(prob, traj, H, f, A, lb, ub);

    // Solve QP: minimize 0.5 x^T H x + f^T x  s.t. lb <= A x <= ub
    // Replace with call to your chosen solver.
    // ...

    // Update traj from solution vector x
    // ...
  }

  return 0;
}
      
