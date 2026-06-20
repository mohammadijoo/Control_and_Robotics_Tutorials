#include <Eigen/Dense>
#include <vector>
#include <random>
#include <iostream>

struct Contact {
  Eigen::Vector3d p;   // position
  Eigen::Vector3d n;   // unit normal
  double mu;           // friction coefficient
};

std::vector<Eigen::Vector3d>
discretizeFrictionCone(const Eigen::Vector3d& n, double mu, int k) {
  Eigen::Vector3d nn = n.normalized();
  Eigen::Vector3d tmp(1.0, 0.0, 0.0);
  if (std::abs(tmp.dot(nn)) > 0.9) {
    tmp = Eigen::Vector3d(0.0, 1.0, 0.0);
  }
  Eigen::Vector3d t1 = nn.cross(tmp).normalized();
  Eigen::Vector3d t2 = nn.cross(t1);

  std::vector<Eigen::Vector3d> dirs;
  dirs.reserve(k);
  const double twoPi = 2.0 * M_PI;
  for (int i = 0; i < k; ++i) {
    double theta = twoPi * double(i) / double(k);
    Eigen::Vector3d t = std::cos(theta) * t1 + std::sin(theta) * t2;
    Eigen::Vector3d f = nn + mu * t;
    f.normalize();
    dirs.push_back(f);
  }
  return dirs;
}

Eigen::MatrixXd buildWrenchSet(const std::vector<Contact>& contacts,
                               int k, double wrenchScale = 1.0) {
  int M = int(contacts.size()) * k;
  Eigen::MatrixXd W(M, 6);
  int row = 0;
  for (const auto& c : contacts) {
    auto dirs = discretizeFrictionCone(c.n, c.mu, k);
    for (const auto& fdir : dirs) {
      Eigen::Vector3d f = wrenchScale * fdir;
      Eigen::Vector3d m = c.p.cross(f);
      W.row(row).segment<3>(0) = f;
      W.row(row).segment<3>(3) = m;
      ++row;
    }
  }
  return W;
}

Eigen::MatrixXd sampleUnitDirections(int dim, int numSamples) {
  std::mt19937 rng(42);
  std::normal_distribution<double> N(0.0, 1.0);
  Eigen::MatrixXd U(numSamples, dim);
  for (int i = 0; i < numSamples; ++i) {
    for (int j = 0; j < dim; ++j) {
      U(i, j) = N(rng);
    }
    U.row(i).normalize();
  }
  return U;
}

double epsilonQuality(const Eigen::MatrixXd& W, int numDirections) {
  if (W.rows() == 0) return 0.0;
  int M = int(W.rows());
  Eigen::MatrixXd U = sampleUnitDirections(6, numDirections); // (Nd,6)
  Eigen::MatrixXd proj = U * W.transpose(); // (Nd,M)
  double eps = std::numeric_limits<double>::infinity();
  for (int i = 0; i < proj.rows(); ++i) {
    double s = proj.row(i).maxCoeff();
    if (s < eps) eps = s;
  }
  return eps;
}

int main() {
  std::vector<Contact> contacts;
  Contact c1;
  c1.p = Eigen::Vector3d(0.05, 0.0, 0.0);
  c1.n = Eigen::Vector3d(-1.0, 0.0, 0.0).normalized();
  c1.mu = 0.7;
  contacts.push_back(c1);

  Contact c2;
  c2.p = Eigen::Vector3d(-0.05, 0.04, 0.0);
  c2.n = Eigen::Vector3d(1.0, -1.0, 0.0).normalized();
  c2.mu = 0.7;
  contacts.push_back(c2);

  Contact c3;
  c3.p = Eigen::Vector3d(-0.05, -0.04, 0.0);
  c3.n = Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
  c3.mu = 0.7;
  contacts.push_back(c3);

  Eigen::MatrixXd W = buildWrenchSet(contacts, 8);
  double q = epsilonQuality(W, 512);
  std::cout << "Approx epsilon quality: " << q << std::endl;
  return 0;
}
      
