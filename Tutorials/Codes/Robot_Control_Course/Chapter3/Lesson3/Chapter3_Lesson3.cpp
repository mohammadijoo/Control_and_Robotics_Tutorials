
#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Matrix2d;

struct TwoLinkParams {
  double l1, l2;
  double m1, m2;
  double I1, I2;
  double g;
};

Matrix2d M_matrix(const Vector2d& q, const TwoLinkParams& p) {
  double q2 = q(1);
  double c2 = std::cos(q2);
  double m11 = p.I1 + p.I2 + p.m1*(p.l1*p.l1)/4.0
               + p.m2*(p.l1*p.l1 + (p.l2*p.l2)/4.0 + p.l1*p.l2*c2);
  double m12 = p.I2 + p.m2*((p.l2*p.l2)/4.0 + 0.5*p.l1*p.l2*c2);
  double m22 = p.I2 + p.m2*(p.l2*p.l2)/4.0;
  Matrix2d M;
  M << m11, m12,
        m12, m22;
  return M;
}

Matrix2d C_matrix(const Vector2d& q, const Vector2d& qd, const TwoLinkParams& p) {
  double q2 = q(1);
  double q1d = qd(0);
  double q2d = qd(1);
  double s2 = std::sin(q2);
  double c11 = -p.m2*p.l1*p.l2*s2*q2d;
  double c12 = -p.m2*p.l1*p.l2*s2*(q1d + q2d);
  double c21 =  p.m2*p.l1*p.l2*s2*q1d;
  double c22 = 0.0;
  Matrix2d C;
  C << c11, c12,
        c21, c22;
  return C;
}

Vector2d g_vector(const Vector2d& q, const TwoLinkParams& p) {
  double q1 = q(0);
  double q2 = q(1);
  double g1 = (p.m1*p.l1/2.0 + p.m2*p.l1)*p.g*std::cos(q1)
              + p.m2*p.l2/2.0*p.g*std::cos(q1 + q2);
  double g2 = p.m2*p.l2/2.0*p.g*std::cos(q1 + q2);
  return Vector2d(g1, g2);
}

int main() {
  TwoLinkParams trueP{1.0, 1.0, 3.0, 2.0, 0.2, 0.1, 9.81};
  TwoLinkParams nomP {1.0, 1.0, 2.5, 1.5, 0.2, 0.1, 9.81};

  Matrix2d Kp = Matrix2d::Zero();
  Matrix2d Kd = Matrix2d::Zero();
  Kp(0,0) = 25.0; Kp(1,1) = 16.0;
  Kd(0,0) = 10.0; Kd(1,1) =  8.0;

  double dt = 0.001;
  Vector2d q   = Vector2d::Zero();
  Vector2d qd  = Vector2d::Zero();

  for (int k = 0; k < 10000; ++k) {
    double t = k*dt;

    // Example desired trajectory (replace with a function as in Python)
    Vector2d qd_d(0.5*std::sin(0.5*t), 0.3*std::sin(0.5*t));
    Vector2d qd1_d(0.5*0.5*std::cos(0.5*t), 0.3*0.5*std::cos(0.5*t));
    Vector2d qd2_d(-0.5*0.5*0.5*std::sin(0.5*t), -0.3*0.5*0.5*std::sin(0.5*t));

    Vector2d e  = q  - qd_d;
    Vector2d ed = qd - qd1_d;

    Matrix2d Mn = M_matrix(q, nomP);
    Matrix2d Cn = C_matrix(q, qd, nomP);
    Vector2d gn = g_vector(q, nomP);

    Vector2d v   = qd2_d - Kd*ed - Kp*e;
    Vector2d tau = Mn*v + Cn*qd + gn;

    Matrix2d Mr = M_matrix(q, trueP);
    Matrix2d Cr = C_matrix(q, qd, trueP);
    Vector2d gr = g_vector(q, trueP);

    Vector2d qdd = Mr.ldlt().solve(tau - Cr*qd - gr);

    // simple Euler integration
    q  += dt*qd;
    qd += dt*qdd;
  }

  std::cout << "Final q: " << q.transpose() << std::endl;
  return 0;
}
