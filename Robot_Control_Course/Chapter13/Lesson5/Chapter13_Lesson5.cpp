
#include <Eigen/Dense>

// Global parameters (for illustration)
const double gamma_joint = 5.0;
const double gamma_ws    = 5.0;
const double lambda_slack = 1e3;

Eigen::Vector2d forwardKinematics(const Eigen::Vector2d& q);
Eigen::Matrix<double, 2, 2> jacobian(const Eigen::Vector2d& q);

// Solve QP: min 0.5 x^T H x + f^T x  s.t. A x <= b
Eigen::VectorXd solveQP(const Eigen::MatrixXd& H,
                        const Eigen::VectorXd& f,
                        const Eigen::MatrixXd& A,
                        const Eigen::VectorXd& b);

void buildCBFConstraints(const Eigen::Vector2d& q,
                         Eigen::MatrixXd& A,
                         Eigen::VectorXd& b)
{
    const int n = 2;
    const int m = 2 * n + 1; // 2 constraints per joint + workspace
    A.resize(m, n);
    b.resize(m);

    Eigen::Vector2d q_min(-M_PI / 2.0, -M_PI / 2.0);
    Eigen::Vector2d q_max( M_PI / 2.0,  M_PI / 2.0);

    int row = 0;
    for (int i = 0; i < n; ++i)
    {
        // Lower limit: q_i - q_min_i >= 0
        double h_min = q(i) - q_min(i);
        Eigen::Vector2d e_i = Eigen::Vector2d::Zero();
        e_i(i) = 1.0;

        // e_i^T v + gamma_joint * h_min >= 0  =>  -e_i^T v <= gamma_joint * h_min
        A.row(row) = -e_i.transpose();
        b(row) = gamma_joint * h_min;
        ++row;

        // Upper limit: q_max_i - q_i >= 0
        double h_max = q_max(i) - q(i);
        // -e_i^T v + gamma_joint * h_max >= 0  =>  e_i^T v <= gamma_joint * h_max
        A.row(row) = e_i.transpose();
        b(row) = gamma_joint * h_max;
        ++row;
    }

    // Workspace constraint
    Eigen::Vector2d c_obs(0.5, 0.5);
    double R_obs = 0.3;

    Eigen::Vector2d p = forwardKinematics(q);
    Eigen::Matrix<double, 2, 2> J = jacobian(q);
    Eigen::Vector2d diff = p - c_obs;
    double h_ws = diff.squaredNorm() - R_obs * R_obs;

    // 2 diff^T J v + gamma_ws * h_ws >= 0
    // => -(2 diff^T J) v <= gamma_ws * h_ws
    Eigen::RowVector2d a_ws = -2.0 * diff.transpose() * J;
    A.row(row) = a_ws;
    b(row) = gamma_ws * h_ws;
}

Eigen::Vector2d safetyFilter(const Eigen::Vector2d& q,
                             const Eigen::Vector2d& v_nom)
{
    const int n = 2;
    Eigen::MatrixXd A_cbf;
    Eigen::VectorXd b_cbf;
    buildCBFConstraints(q, A_cbf, b_cbf);
    int m = static_cast<int>(b_cbf.size());

    // Decision x = [v; delta] in R^{n+1}
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n + 1, n + 1);
    H.block(0, 0, n, n) = Eigen::Matrix2d::Identity();
    H(n, n) = lambda_slack;

    Eigen::VectorXd f = Eigen::VectorXd::Zero(n + 1);
    f.segment(0, n) = -v_nom;

    // Inequalities: A_cbf v - delta <= b_cbf   and   -delta <= 0
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m + 1, n + 1);
    Eigen::VectorXd b(m + 1);

    A.block(0, 0, m, n) = A_cbf;
    A.block(0, n, m, 1) = -Eigen::VectorXd::Ones(m);
    b.segment(0, m) = b_cbf;

    A(m, n) = -1.0;
    b(m) = 0.0;

    Eigen::VectorXd x_opt = solveQP(H, f, A, b);
    return x_opt.segment(0, n);
}
