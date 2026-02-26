#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main() {
    double zeta = 0.4;
    double wn   = 5.0;
    double a    = 20.0;

    // Third-order denominator: (s + a)(s^2 + 2 zeta wn s + wn^2)
    // For simulation, we pick a companion form realization.
    // x_dot = A x + B u, y = C x

    Eigen::Matrix3d A;
    Eigen::Vector3d B;
    Eigen::RowVector3d C;

    // Coefficients of s^3 + alpha2 s^2 + alpha1 s + alpha0
    double alpha2 = 2.0*zeta*wn + a;
    double alpha1 = wn*wn + 2.0*zeta*wn*a;
    double alpha0 = wn*wn*a;

    // Companion matrix (controllable canonical form)
    A <<  0.0,     1.0,     0.0,
           0.0,     0.0,     1.0,
          -alpha0, -alpha1, -alpha2;

    B << 0.0, 0.0, 1.0;
    // Choose C so that DC gain is approximately 1 (here we set C = [c0 c1 c2])
    // For simplicity, pick C = [0 0 k]; in practice, compute from transfer function.
    double kp = a;
    double k  = kp * wn*wn;
    C << 0.0, 0.0, k;

    double dt = 0.0005;
    double t_end = 4.0;
    int steps = static_cast<int>(t_end / dt);

    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    double u = 1.0;  // unit step
    for (int kstep = 0; kstep <= steps; ++kstep) {
        double t = kstep * dt;
        double y = (C * x)(0);

        std::cout << t << " " << y << std::endl;

        Eigen::Vector3d xdot = A * x + B * u;
        x += dt * xdot;
    }
    return 0;
}
