#include <Eigen/Dense>

struct PendulumParams {
    double m;
    double l;
    double g;
};

// Compute qddot for a simple pendulum using Lagrange-Euler EoM
double pendulum_accel(double q, double qdot, double tau,
                      const PendulumParams& p)
{
    double M = p.m * p.l * p.l;          // inertia term
    double G = p.m * p.g * p.l * std::sin(q); // gravity torque
    double Cqdot = 0.0;                  // no Coriolis term for 1-DOF

    double qddot = (tau - Cqdot - G) / M;
    return qddot;
}

// Sketch of a multi-DOF signature:
// Eigen::VectorXd robot_accel(const Eigen::VectorXd& q,
//                             const Eigen::VectorXd& qdot,
//                             const Eigen::VectorXd& tau)
// {
//     // Compute M(q), C(q,qdot), g(q) from link inertias and Jacobians
//     // M, C, g can be assembled using a robotics library like RBDL:
//     //
//     //   RigidBodyDynamics::Model model;
//     //   // ... build model (masses, COMs, inertias, joint types)
//     //   Eigen::VectorXd qddot = Eigen::VectorXd::Zero(q.size());
//     //   RigidBodyDynamics::InverseDynamics(model, q, qdot, qddot, tau);
//     //
//     // For Lagrange-Euler, you would explicitly build M, C, g
//     // and compute qddot = M^{-1} (tau - C qdot - g).
// }
      
