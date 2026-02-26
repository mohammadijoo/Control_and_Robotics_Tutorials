// Chapter6_Lesson3.cpp
// Autonomous Mobile Robots (Control Engineering) - Chapter 6, Lesson 3
// Motion Models vs Sensor Models: A discrete Bayes-filter demonstration (1D grid)
//
// Build example (Linux/macOS):
//   g++ -O2 -std=c++17 Chapter6_Lesson3.cpp -I /usr/include/eigen3 -o Chapter6_Lesson3
//
// Notes:
// - Uses Eigen for vector math.
// - For robotics ecosystems, Eigen is pervasive (ROS/ROS2, SLAM toolkits, etc.).

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

struct OneDWorld {
  int N = 121;
  double dx = 0.1;
  double beacon_x = 6.0;

  Eigen::VectorXd xs() const {
    Eigen::VectorXd x(N);
    for (int i = 0; i < N; ++i) x(i) = i * dx;
    return x;
  }
};

static inline double gaussian_pdf(double x, double mu, double sigma) {
  if (sigma <= 0.0) throw std::runtime_error("sigma must be > 0");
  const double z = (x - mu) / sigma;
  return (1.0 / (std::sqrt(2.0 * M_PI) * sigma)) * std::exp(-0.5 * z * z);
}

static inline Eigen::VectorXd normalize(const Eigen::VectorXd& p) {
  const double s = p.sum();
  if (s <= 1e-15) throw std::runtime_error("Probability mass nearly zero.");
  return p / s;
}

Eigen::VectorXd motion_predict(const Eigen::VectorXd& bel,
                               double u,
                               double sigma_u,
                               const OneDWorld& world) {
  Eigen::VectorXd xs = world.xs();
  Eigen::VectorXd bel_bar = Eigen::VectorXd::Zero(world.N);

  // bel_bar(x) = sum_{x'} N(x; x' + u, sigma_u^2) bel(x')
  for (int i = 0; i < world.N; ++i) {
    const double x_prev = xs(i);
    const double mu = x_prev + u;
    for (int j = 0; j < world.N; ++j) {
      const double x = xs(j);
      bel_bar(j) += bel(i) * gaussian_pdf(x, mu, sigma_u);
    }
  }
  return normalize(bel_bar);
}

Eigen::VectorXd sensor_update(const Eigen::VectorXd& bel_bar,
                              double z,
                              double sigma_z,
                              const OneDWorld& world) {
  Eigen::VectorXd xs = world.xs();
  Eigen::VectorXd post(world.N);

  // p(z|x) = N(z; |x - beacon|, sigma_z^2)
  for (int i = 0; i < world.N; ++i) {
    const double expected = std::abs(xs(i) - world.beacon_x);
    const double likelihood = gaussian_pdf(z, expected, sigma_z);
    post(i) = bel_bar(i) * likelihood;
  }
  return normalize(post);
}

double entropy_nats(const Eigen::VectorXd& p) {
  double H = 0.0;
  for (int i = 0; i < p.size(); ++i) {
    const double pi = p(i);
    if (pi > 0.0) H -= pi * std::log(pi);
  }
  return H;
}

int main() {
  OneDWorld world;

  Eigen::VectorXd bel = Eigen::VectorXd::Ones(world.N);
  bel = normalize(bel);

  std::vector<double> u_seq = {0.5, 0.5, 0.5, 0.5};
  std::vector<double> z_seq = {5.5, 5.0, 4.5, 4.0};

  const double sigma_u = 0.25;
  const double sigma_z = 0.35;

  std::cout << "t |  MAP estimate (m) |  belief entropy (nats)\n";
  std::cout << "--+-------------------+----------------------\n";

  for (size_t t = 0; t < u_seq.size(); ++t) {
    Eigen::VectorXd bel_bar = motion_predict(bel, u_seq[t], sigma_u, world);
    bel = sensor_update(bel_bar, z_seq[t], sigma_z, world);

    Eigen::VectorXd xs = world.xs();
    Eigen::Index idx;
    bel.maxCoeff(&idx);
    const double x_map = xs(idx);

    std::cout << (t + 1) << " | " << std::setw(17) << std::fixed << std::setprecision(3)
              << x_map << " | " << std::setw(20) << std::setprecision(6)
              << entropy_nats(bel) << "\n";
  }

  return 0;
}
