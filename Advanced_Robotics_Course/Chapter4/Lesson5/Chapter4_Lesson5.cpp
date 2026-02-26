#include <Eigen/Dense>
#include <vector>
#include <iostream>

// Waypoint trajectory for an n-DOF robot
struct Trajectory {
    std::vector<Eigen::VectorXd> q; // size N_plus_1, each of dim n
};

// Build smoothness cost and gradient using second differences
double smoothnessCostAndGrad(const Trajectory& traj,
                             std::vector<Eigen::VectorXd>& grad) {
    const int N_plus_1 = static_cast<int>(traj.q.size());
    const int n = static_cast<int>(traj.q[0].size());
    grad.assign(N_plus_1, Eigen::VectorXd::Zero(n));

    double cost = 0.0;
    for (int k = 1; k < N_plus_1 - 1; ++k) {
        Eigen::VectorXd ddq = traj.q[k + 1] - 2.0 * traj.q[k] + traj.q[k - 1];
        cost += 0.5 * ddq.squaredNorm();

        grad[k - 1] += -ddq;
        grad[k]     +=  2.0 * ddq;
        grad[k + 1] += -ddq;
    }
    return cost;
}

// Collision interface (to be implemented using, e.g., FCL)
struct CollisionResult {
    double cost;
    std::vector<Eigen::VectorXd> grad; // same size as traj.q
};

// Placeholder: in a real system, call FCL or another library
CollisionResult collisionCostAndGrad(const Trajectory& traj) {
    CollisionResult res;
    const int N_plus_1 = static_cast<int>(traj.q.size());
    const int n = static_cast<int>(traj.q[0].size());
    res.cost = 0.0;
    res.grad.assign(N_plus_1, Eigen::VectorXd::Zero(n));

    // Example: assume we have a function
    //   querySignedDistanceAndDirection(q, d, normal)
    // for each waypoint that returns signed distance and workspace normal.
    //
    // Here we just leave a stub to show how the gradient would be used.
    //
    // double clearance = 0.05;
    // for (int k = 1; k < N_plus_1 - 1; ++k) {
    //     double d;
    //     Eigen::VectorXd grad_q = Eigen::VectorXd::Zero(n);
    //     // fill d and grad_q using collision library + robot Jacobians
    //     if (d < clearance) {
    //         double phi = 0.5 * (clearance - d) * (clearance - d);
    //         res.cost += phi;
    //         double dphi_dd = -(clearance - d);
    //         res.grad[k] += dphi_dd * grad_q;
    //     }
    // }

    return res;
}

int main() {
    const int N = 40;
    const int n = 2;

    Trajectory traj;
    traj.q.resize(N + 1);
    Eigen::Vector2d q_start(-0.5, 1.0);
    Eigen::Vector2d q_goal(1.0, -0.5);

    for (int k = 0; k <= N; ++k) {
        double alpha = static_cast<double>(k) / N;
        traj.q[k] = (1.0 - alpha) * q_start + alpha * q_goal;
    }

    double lambda_smooth = 1.0;
    double lambda_col = 10.0;
    double alpha_step = 0.01;

    std::vector<Eigen::VectorXd> grad_s, grad_total;
    for (int it = 0; it < 100; ++it) {
        double Js = smoothnessCostAndGrad(traj, grad_s);
        CollisionResult col = collisionCostAndGrad(traj);

        grad_total = grad_s;
        for (int k = 0; k <= N; ++k) {
            grad_total[k] =
                lambda_smooth * grad_s[k] + lambda_col * col.grad[k];
        }

        // Gradient descent on internal nodes
        for (int k = 1; k < N; ++k) {
            traj.q[k] -= alpha_step * grad_total[k];
        }

        std::cout << "Iter " << it
                  << ": Js = " << Js
                  << ", Jc = " << col.cost << std::endl;
    }

    return 0;
}
      
