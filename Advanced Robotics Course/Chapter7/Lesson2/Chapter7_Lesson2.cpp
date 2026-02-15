#include <Eigen/Dense>
#include <vector>
#include <cmath>

struct Contact2D {
    Eigen::Vector2d p;  // position
    Eigen::Vector2d n;  // inward unit normal
};

std::vector<Eigen::Vector2d> frictionDirections2D(
    const Eigen::Vector2d& n, double mu)
{
    std::vector<Eigen::Vector2d> dirs;
    Eigen::Vector2d t(-n.y(), n.x());  // tangent
    double phi = std::atan(mu);
    Eigen::Vector2d d1 = std::cos(phi) * n + std::sin(phi) * t;
    Eigen::Vector2d d2 = std::cos(phi) * n - std::sin(phi) * t;
    dirs.push_back(d1);
    dirs.push_back(d2);
    return dirs;
}

Eigen::MatrixXd buildWrenchMatrix2D(
    const std::vector<Contact2D>& contacts, double mu)
{
    std::vector<Eigen::Vector3d> wcols;
    for (const auto& c : contacts) {
        auto dirs = frictionDirections2D(c.n, mu);
        for (const auto& d : dirs) {
            double fx = d.x();
            double fy = d.y();
            double x = c.p.x();
            double y = c.p.y();
            double m = x * fy - y * fx;
            wcols.emplace_back(fx, fy, m);
        }
    }
    Eigen::MatrixXd G(3, static_cast<int>(wcols.size()));
    for (int j = 0; j < static_cast<int>(wcols.size()); ++j) {
        G.col(j) = wcols[j];
    }
    return G;
}

// Pseudo-code: call external LP solver to check feasibility:
//   minimize 0
//   subject to G * alpha = 0, sum(alpha) = 1, alpha_i >= eps.
//
// bool isForceClosure2D(const std::vector<Contact2D>& contacts,
//                       double mu, double eps = 1e-4)
// {
//     Eigen::MatrixXd G = buildWrenchMatrix2D(contacts, mu);
//     if (Eigen::FullPivLU<Eigen::MatrixXd>(G).rank() < 3)
//         return false;
//
//     int M = static_cast<int>(G.cols());
//     // Build LP matrices for your solver of choice.
//     // A_eq = [G; 1^T], b_eq = [0; 1],
//     // bounds: alpha_i in [eps, +inf).
//     // Solve and return true if feasible.
// }
      
