#include <Eigen/Dense>
#include <vector>
#include <cmath>

struct TrajectoryOptimizationProblem {
  int N;
  double T;
  double h;

  TrajectoryOptimizationProblem(int N_, double T_)
      : N(N_), T(T_), h(T_ / N_) {}

  // x = [p, v], u = [a]
  Eigen::Vector2d dynamics(const Eigen::Vector2d& x,
                           double u) const {
    Eigen::Vector2d xdot;
    xdot(0) = x(1);
    xdot(1) = u;
    return xdot;
  }

  // Decision vector w layout:
  // [x0 (2), u0 (1), x1 (2), u1 (1), ..., u_{N-1} (1), xN (2)]
  // This helper extracts state and control at step k.
  void getIndex(int k, int& idx_xk, int& idx_uk) const {
    // x0 at index 0
    // pattern: x0, u0, x1, u1, ...
    idx_xk = 3 * k;      // each stage adds 3 variables: 2 (x) + 1 (u)
    idx_uk = idx_xk + 2;
  }

  double cost(const Eigen::VectorXd& w) const {
    double J = 0.0;
    for (int k = 0; k < N; ++k) {
      int idx_xk, idx_uk;
      getIndex(k, idx_xk, idx_uk);
      double ak = w(idx_uk);
      J += 0.5 * h * ak * ak;
    }
    return J;
  }

  // Computes dynamic constraint residuals: x_{k+1} - (x_k + h * f(x_k, u_k))
  void dynamicsResiduals(const Eigen::VectorXd& w,
                         Eigen::VectorXd& g) const {
    g.setZero(2 * N);
    for (int k = 0; k < N; ++k) {
      int idx_xk, idx_uk;
      getIndex(k, idx_xk, idx_uk);
      Eigen::Vector2d xk = w.segment<2>(idx_xk);
      double uk = w(idx_uk);
      Eigen::Vector2d xkp1 = w.segment<2>(idx_xk + 3);  // x_{k+1}
      Eigen::Vector2d x_next_euler = xk + h * dynamics(xk, uk);
      g.segment<2>(2 * k) = xkp1 - x_next_euler;
    }
  }
};

int main() {
  int N = 40;
  double T = 2.0;
  TrajectoryOptimizationProblem prob(N, T);

  // Example: allocate decision vector w with some initial guess.
  Eigen::VectorXd w(3 * N + 2);
  w.setZero();

  // Set initial state and final state through bounds in the solver.
  // Those bounds are not shown here, but conceptually they are:
  // x0 = [p0, v0], xN = [pf, vf].

  // Evaluate cost and dynamics residuals for debugging
  double J = prob.cost(w);
  Eigen::VectorXd g(2 * N);
  prob.dynamicsResiduals(w, g);

  return 0;
}
      
