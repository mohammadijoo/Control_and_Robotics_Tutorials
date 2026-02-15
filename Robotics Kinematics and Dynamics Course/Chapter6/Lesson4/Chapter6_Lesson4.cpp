#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Matrix2d;

Vector2d fk2R(const Vector2d& q, double l1, double l2) {
    double q1 = q(0);
    double q2 = q(1);
    double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
    return Vector2d(x, y);
}

Matrix2d jacobian2R(const Vector2d& q, double l1, double l2) {
    double q1 = q(0);
    double q2 = q(1);
    double s1  = std::sin(q1);
    double c1  = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Matrix2d J;
    J(0,0) = -l1 * s1 - l2 * s12;
    J(0,1) = -l2 * s12;
    J(1,0) =  l1 * c1 + l2 * c12;
    J(1,1) =  l2 * c12;
    return J;
}

bool ikNewton2R(const Vector2d& xd,
                double l1, double l2,
                Vector2d& q,
                double tol = 1e-6,
                int maxIter = 50,
                double alpha = 1.0) {
    for (int k = 0; k < maxIter; ++k) {
        Vector2d x = fk2R(q, l1, l2);
        Vector2d r = x - xd;
        double err = r.norm();
        if (err < tol) {
            return true;
        }
        Matrix2d J = jacobian2R(q, l1, l2);
        Eigen::FullPivLU<Matrix2d> lu(J);
        if (!lu.isInvertible()) {
            return false;  // near singular
        }
        Vector2d dq = lu.solve(-r);
        q += alpha * dq;
    }
    return false;
}

bool ikDLS2R(const Vector2d& xd,
             double l1, double l2,
             Vector2d& q,
             double tol = 1e-6,
             int maxIter = 50,
             double alpha = 1.0,
             double lambda = 1e-2) {
    for (int k = 0; k < maxIter; ++k) {
        Vector2d x = fk2R(q, l1, l2);
        Vector2d r = x - xd;
        double err = r.norm();
        if (err < tol) {
            return true;
        }
        Matrix2d J   = jacobian2R(q, l1, l2);
        Matrix2d JTJ = J.transpose() * J;
        Matrix2d A   = JTJ + (lambda * lambda) * Matrix2d::Identity();
        Vector2d g   = J.transpose() * r;
        Vector2d dq  = -A.ldlt().solve(g);
        q += alpha * dq;
    }
    return false;
}

int main() {
    double l1 = 1.0, l2 = 0.8;
    Vector2d xd(1.2, 0.5);
    Vector2d q0(0.0, 0.0);
    Vector2d qN = q0;
    Vector2d qD = q0;

    bool okN = ikNewton2R(xd, l1, l2, qN);
    bool okD = ikDLS2R(xd, l1, l2, qD, 1e-6, 50, 1.0, 0.05);

    std::cout << "Newton IK success: " << okN << ", q = "
              << qN.transpose() << std::endl;
    std::cout << "DLS   IK success: " << okD << ", q = "
              << qD.transpose() << std::endl;
    return 0;
}
      
