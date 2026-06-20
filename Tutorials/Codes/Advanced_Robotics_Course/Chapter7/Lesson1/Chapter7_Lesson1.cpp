#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

struct ContactPoint {
  Eigen::Vector3d p_obj;   // contact position in object frame
  Eigen::Vector3d n_obj;   // normal in object frame (unit, into object)
  double mu;               // friction coefficient
};

std::vector<Eigen::Vector3d>
frictionConeGenerators3D(const Eigen::Vector3d& n, double mu, int m_azimuth) {
  if (mu <= 0.0) {
    throw std::runtime_error("mu must be positive");
  }
  // Build orthonormal basis {t1, t2, n}
  Eigen::Vector3d t1;
  if (std::abs(n.x()) > 0.5) {
    t1 = Eigen::Vector3d(-n.y(), n.x(), 0.0);
  } else {
    t1 = Eigen::Vector3d(0.0, -n.z(), n.y());
  }
  t1.normalize();
  Eigen::Vector3d t2 = n.cross(t1);

  double alpha = std::atan(mu);
  std::vector<Eigen::Vector3d> rays;
  rays.reserve(m_azimuth);

  for (int k = 0; k < m_azimuth; ++k) {
    double theta = 2.0 * M_PI * static_cast<double>(k) / m_azimuth;
    // Tangential direction on the cone boundary
    Eigen::Vector3d t = std::cos(theta) * t1 + std::sin(theta) * t2;
    Eigen::Vector3d f = std::sin(alpha) * t + std::cos(alpha) * n;
    f.normalize();
    rays.push_back(f);
  }
  return rays;
}

Eigen::MatrixXd contactWrenchGenerators3D(const ContactPoint& cp, int m_azimuth) {
  // Rays are unit directions of force in object frame
  auto rays = frictionConeGenerators3D(cp.n_obj, cp.mu, m_azimuth);
  const int m = static_cast<int>(rays.size());
  Eigen::MatrixXd W(6, m);
  for (int j = 0; j < m; ++j) {
    Eigen::Vector3d f = rays[j];         // assume scaled later by nonneg coeffs
    Eigen::Vector3d tau = cp.p_obj.cross(f);
    W.block<3,1>(0, j) = f;
    W.block<3,1>(3, j) = tau;
  }
  return W; // each column is a wrench generator
}
      
