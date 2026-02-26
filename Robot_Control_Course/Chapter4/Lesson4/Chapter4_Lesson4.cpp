
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd pseudoinverse(const MatrixXd& J, double damping = 0.0)
{
    // Right pseudoinverse: J# = J^T (J J^T + λ^2 I)^(-1)
    const int m = J.rows();
    const int n = J.cols();
    MatrixXd JJt = J * J.transpose();
    if (damping > 0.0) {
        JJt += (damping * damping) * MatrixXd::Identity(m, m);
    }
    MatrixXd JJt_inv = JJt.inverse();
    return J.transpose() * JJt_inv;
}

VectorXd twoTaskHierarchy(const MatrixXd& J1,
                          const VectorXd& dx1,
                          const MatrixXd& J2,
                          const VectorXd& dx2,
                          double damping1 = 0.0,
                          double damping2 = 0.0)
{
    const int n = J1.cols();
    MatrixXd I = MatrixXd::Identity(n, n);

    MatrixXd J1_pinv = pseudoinverse(J1, damping1);
    VectorXd dq1 = J1_pinv * dx1;
    MatrixXd N1 = I - J1_pinv * J1;

    MatrixXd J2_bar = J2 * N1;
    MatrixXd J2_bar_pinv = pseudoinverse(J2_bar, damping2);
    VectorXd dx2_tilde = dx2 - J2 * dq1;
    VectorXd dq2_null = J2_bar_pinv * dx2_tilde;

    VectorXd dq = dq1 + N1 * dq2_null;
    return dq;
}

int main()
{
    const int n = 7;
    MatrixXd J1(3, n);
    MatrixXd J2(n, n);
    J1.setRandom();
    J2.setIdentity();

    VectorXd dx1(3);
    dx1 << 0.1, -0.05, 0.0;

    VectorXd q_pref = VectorXd::Zero(n);
    VectorXd q_current(n);
    q_current << 0.2, -0.3, 0.1, 0.0, 0.2, -0.1, 0.1;

    double k_posture = 0.5;
    VectorXd dx2 = k_posture * (q_pref - q_current);

    VectorXd dq = twoTaskHierarchy(J1, dx1, J2, dx2, 1e-3, 1e-3);
    std::cout << "dq command:\n" << dq.transpose() << std::endl;
    return 0;
}
