
#include <iostream>
#include <random>
#include <Eigen/Dense>

struct JointParams {
  double I;    // inertia
  double b;    // viscous friction
  double mgl;  // gravity torque coefficient
};

JointParams sampleParams(const JointParams& nominal,
                         const JointParams& delta_max,
                         std::mt19937& gen) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  JointParams theta;
  theta.I   = nominal.I   + delta_max.I   * dist(gen);
  theta.b   = nominal.b   + delta_max.b   * dist(gen);
  theta.mgl = nominal.mgl + delta_max.mgl * dist(gen);
  return theta;
}

double torqueModel(const JointParams& theta, double q, double qdot, double qddot) {
  // tau = I qddot + b qdot + mgl sin(q)
  return theta.I * qddot + theta.b * qdot + theta.mgl * std::sin(q);
}

int main() {
  JointParams nominal{0.5, 0.05, 1.0};
  JointParams delta_max{0.1, 0.02, 0.2};

  std::mt19937 gen(42);

  double q = 0.3;
  double qdot = 0.1;
  double qddot = 0.0;

  // Nominal torque
  double tau_hat = torqueModel(nominal, q, qdot, qddot);

  // Sample one realization of parametric uncertainty
  JointParams theta_true = sampleParams(nominal, delta_max, gen);
  double tau_true = torqueModel(theta_true, q, qdot, qddot);

  double delta_tau = tau_true - tau_hat;
  std::cout << "Nominal tau = " << tau_hat
            << ", true tau = " << tau_true
            << ", mismatch = " << delta_tau << std::endl;

  return 0;
}
