// Chapter6_Lesson4.cpp
// Independence Assumptions and Their Limits — correlated-sensor Bayesian fusion (C++)
//
// Demonstrates overconfidence when measurement correlation is ignored.
//
// Dependencies: Eigen3 (common in robotics).
// Compile (example):
//   g++ -O2 -std=c++17 Chapter6_Lesson4.cpp -I /usr/include/eigen3 -o ch6l4
// Run:
//   ./ch6l4

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

struct Posterior {
  double mean;
  double var;
};

Posterior posterior_correlated(double mu0, double sigma0, const Eigen::Vector2d& y, double sigma, double rho) {
  Eigen::Matrix2d Sigma;
  Sigma << 1.0, rho,
           rho, 1.0;
  Sigma *= (sigma * sigma);

  Eigen::Matrix2d Sinv = Sigma.inverse();
  Eigen::Vector2d H;
  H << 1.0, 1.0;

  double info_meas = H.transpose() * Sinv * H;                 // scalar
  double post_var = 1.0 / (1.0/(sigma0*sigma0) + info_meas);   // scalar

  double info_vec = H.transpose() * Sinv * y;                  // scalar
  double post_mean = post_var * (mu0/(sigma0*sigma0) + info_vec);

  return {post_mean, post_var};
}

Posterior posterior_independent(double mu0, double sigma0, const Eigen::Vector2d& y, double sigma) {
  // rho=0 -> Sigma = sigma^2 I; precision additivity
  double post_var = 1.0 / (1.0/(sigma0*sigma0) + 2.0/(sigma*sigma));
  double post_mean = post_var * (mu0/(sigma0*sigma0) + (y(0) + y(1))/(sigma*sigma));
  return {post_mean, post_var};
}

int main() {
  double mu0 = 0.0;
  double sigma0 = 2.0;
  double sigma = 1.0;
  Eigen::Vector2d y(1.0, 1.2);

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Prior: x ~ N(mu0, sigma0^2) with mu0=" << mu0 << ", sigma0=" << sigma0 << "\n";
  std::cout << "Measurements: y1=" << y(0) << ", y2=" << y(1) << ", sigma=" << sigma << "\n\n";

  for (double rho : {0.0, 0.3, 0.6, 0.9}) {
    Posterior pc = posterior_correlated(mu0, sigma0, y, sigma, rho);
    Posterior pi = posterior_independent(mu0, sigma0, y, sigma);

    double ratio = pi.var / pc.var;
    std::cout << "rho=" << rho << ":\n";
    std::cout << "  Correct correlated posterior: mean=" << pc.mean << ", var=" << pc.var << "\n";
    std::cout << "  Indep. assumption posterior:  mean=" << pi.mean << ", var=" << pi.var << "\n";
    std::cout << "  Variance ratio (indep/correct) = " << ratio << " ( < 1 means overconfident )\n\n";
  }
  return 0;
}
