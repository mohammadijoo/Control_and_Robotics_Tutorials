
#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Matrix2d;

Vector2d forwardKinematics(const Vector2d& q);
Matrix2d jacobian(const Vector2d& q);
Matrix2d jacobianDot(const Vector2d& q, const Vector2d& qdot);
Matrix2d M_matrix(const Vector2d& q);
Vector2d C_vector(const Vector2d& q, const Vector2d& qdot);
Vector2d g_vector(const Vector2d& q);

Vector2d x_des(double t);
Vector2d xdot_des(double t);
Vector2d xddot_des(double t);

int main() {
    double dt = 0.001;
    double t_final = 5.0;

    Matrix2d Kp = Matrix2d::Zero();
    Matrix2d Kd = Matrix2d::Zero();
    Kp(0,0) = 100.0; Kp(1,1) = 100.0;
    Kd(0,0) = 20.0;  Kd(1,1) = 20.0;

    Vector2d q(0.0, 0.0);
    Vector2d qdot(0.0, 0.0);

    int N = static_cast<int>(t_final / dt);
    for (int k = 0; k < N; ++k) {
        double t = k * dt;

        Vector2d x   = forwardKinematics(q);
        Matrix2d J   = jacobian(q);
        Vector2d xdot = J * qdot;
        Matrix2d Jdot = jacobianDot(q, qdot);

        Vector2d xd    = x_des(t);
        Vector2d xd_dot = xdot_des(t);
        Vector2d xdd    = xddot_des(t);

        Vector2d e    = xd - x;
        Vector2d e_dot = xd_dot - xdot;

        Vector2d a_x = xdd + Kd * e_dot + Kp * e;

        Matrix2d Jinv = J.inverse();
        Vector2d qdd_des = Jinv * (a_x - Jdot * qdot);

        Matrix2d M = M_matrix(q);
        Vector2d C = C_vector(q, qdot);
        Vector2d g = g_vector(q);

        Vector2d tau = M * qdd_des + C + g;

        // plant integration (explicit Euler using same model)
        Vector2d qdd = M.ldlt().solve(tau - C - g);
        qdot += dt * qdd;
        q    += dt * qdot;
    }

    std::cout << "Simulation finished." << std::endl;
    return 0;
}
