#include <Eigen/Dense>

// Example dimensions
constexpr int NQ = 7;
constexpr int NX = 2 * NQ;
constexpr int NU = NQ;

// Toy residual MLP with one hidden layer
struct ResidualMLP {
  Eigen::Matrix<double, NX + NU, NX> W1;
  Eigen::Matrix<double, NX, 1> b1;
  Eigen::Matrix<double, NX, NX> W2;
  Eigen::Matrix<double, NX, 1> b2;

  Eigen::Matrix<double, NX, 1> forward(
      const Eigen::Matrix<double, NX, 1> &x,
      const Eigen::Matrix<double, NU, 1> &u) const {
    Eigen::Matrix<double, NX + NU, 1> in;
    in << x, u;
    Eigen::Matrix<double, NX, 1> h = (W1.transpose() * in + b1).array().tanh();
    Eigen::Matrix<double, NX, 1> out = W2.transpose() * h + b2;
    return out;
  }
};

// Placeholder for analytical discrete-time dynamics
Eigen::Matrix<double, NX, 1> f_phys(
    const Eigen::Matrix<double, NX, 1> &x,
    const Eigen::Matrix<double, NU, 1> τ,
    double dt) {
  Eigen::Matrix<double, NX, 1> x_next;
  // Here we just do a dummy integrator; in practice call your rigid-body dynamics
  x_next.head(NQ) = x.head(NQ) + dt * x.tail(NQ);
  x_next.tail(NQ).setZero();
  return x_next;
}

Eigen::Matrix<double, NX, 1> f_hybrid(
    const Eigen::Matrix<double, NX, 1> &x,
    const Eigen::Matrix<double, NU, 1> τ,
    double dt,
    const ResidualMLP &residual) {
  Eigen::Matrix<double, NX, 1> x_phys = f_phys(x, tau, dt);
  Eigen::Matrix<double, NX, 1> r = residual.forward(x, tau);
  return x_phys + r;
}

int main() {
  ResidualMLP res;
  // TODO: load weights into res.W1, res.b1, res.W2, res.b2

  Eigen::Matrix<double, NX, 1> x;
  Eigen::Matrix<double, NU, 1> tau;
  x.setZero();
  tau.setZero();

  double dt = 0.01;
  Eigen::Matrix<double, NX, 1> x_next = f_hybrid(x, tau, dt, res);

  // Use x_next in your controller or simulator
  return 0;
}
      
