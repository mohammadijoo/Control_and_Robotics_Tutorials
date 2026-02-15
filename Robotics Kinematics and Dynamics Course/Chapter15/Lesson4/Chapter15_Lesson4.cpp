#include <iostream>
#include <Eigen/Dense>

struct State {
    double x;
    double y;
    double vx;
    double vy;
};

struct Params {
    double m;
    double g;
    double L;
    double alpha;
    double beta;
};

Eigen::Vector4d baumgarteRHS(const State& s, const Params& p) {
    double x  = s.x;
    double y  = s.y;
    double vx = s.vx;
    double vy = s.vy;

    // Inertia
    Eigen::Matrix2d M = p.m * Eigen::Matrix2d::Identity();
    Eigen::Vector2d h;
    h << 0.0, p.m * p.g;

    // Constraint
    double phi = x * x + y * y - p.L * p.L;

    Eigen::RowVector2d J;
    J << 2.0 * x, 2.0 * y;

    Eigen::RowVector2d Jdot;
    Jdot << 2.0 * vx, 2.0 * vy;

    Eigen::Vector2d qdot;
    qdot << vx, vy;

    // Build K (3x3)
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K.block<2,2>(0,0) = M;
    K.block<2,1>(0,2) = -J.transpose();
    K.block<1,2>(2,0) = J;

    // Right-hand side
    Eigen::Vector3d rhs;
    rhs.segment<2>(0) = -h;
    double rhs_con = -(Jdot * qdot
                       + 2.0 * p.alpha * (J * qdot)
                       + p.beta * p.beta * phi);
    rhs(2) = rhs_con;

    // Solve K * [qddot; lambda] = rhs
    Eigen::Vector3d sol = K.fullPivLu().solve(rhs);
    Eigen::Vector2d qddot = sol.segment<2>(0);

    Eigen::Vector4d ydot;
    ydot << vx, vy, qddot(0), qddot(1);
    return ydot;
}

// Integrate baumgarteRHS with your preferred ODE integrator (e.g., RK4).
      
