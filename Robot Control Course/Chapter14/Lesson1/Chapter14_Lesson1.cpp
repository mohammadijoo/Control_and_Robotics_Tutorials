
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd dampedPseudoInverse(const MatrixXd& J, double lambda)
{
    int m = J.rows();
    int n = J.cols();
    if (m >= n) {
        MatrixXd A = J.transpose() * J +
                     (lambda * lambda) * MatrixXd::Identity(n, n);
        return A.ldlt().solve(J.transpose());
    } else {
        MatrixXd A = J * J.transpose() +
                     (lambda * lambda) * MatrixXd::Identity(m, m);
        return J.transpose() * A.ldlt().solve(MatrixXd::Identity(m, m));
    }
}

MatrixXd nullspaceProjector(const MatrixXd& J, double lambda)
{
    int n = J.cols();
    MatrixXd J_pinv = dampedPseudoInverse(J, lambda);
    MatrixXd I = MatrixXd::Identity(n, n);
    return I - J_pinv * J;
}

VectorXd twoTaskVelocityControl(const MatrixXd& J1,
                                const VectorXd& xdot1_star,
                                const MatrixXd& J2,
                                const VectorXd& xdot2_star,
                                double lambda)
{
    // Task 1
    MatrixXd J1_pinv = dampedPseudoInverse(J1, lambda);
    VectorXd qdot1 = J1_pinv * xdot1_star;

    // Null space of Task 1
    MatrixXd N1 = nullspaceProjector(J1, lambda);

    // Effective Task 2 in null space
    MatrixXd J2_eff = J2 * N1;
    MatrixXd J2_eff_pinv = dampedPseudoInverse(J2_eff, lambda);

    VectorXd residual2 = xdot2_star - J2 * qdot1;
    VectorXd qdot2_corr = N1 * (J2_eff_pinv * residual2);

    VectorXd qdot = qdot1 + qdot2_corr;
    return qdot;
}

int main()
{
    int n_dof = 7;
    int m1 = 3;
    int m2 = 3;

    MatrixXd J1 = MatrixXd::Random(m1, n_dof);
    MatrixXd J2 = MatrixXd::Random(m2, n_dof);

    VectorXd xdot1_star(m1);
    xdot1_star << 0.1, 0.0, -0.05;

    VectorXd xdot2_star(m2);
    xdot2_star << 0.0, 0.05, 0.0;

    double lambda = 1e-3;
    VectorXd qdot = twoTaskVelocityControl(J1, xdot1_star, J2, xdot2_star, lambda);

    std::cout << "qdot = " << qdot.transpose() << std::endl;
    return 0;
}
