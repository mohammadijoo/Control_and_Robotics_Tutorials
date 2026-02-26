#include <vector>
#include <Eigen/Dense>

struct Link {
    double m;
    Eigen::Matrix3d I;
    Eigen::Vector3d r_com;
    char joint_type; // 'R' or 'P'
};

Eigen::VectorXd inverseDynamicsNewtonEuler(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd,
    const Eigen::Vector3d& g,
    const std::vector<Eigen::Matrix3d>& R,
    const std::vector<Eigen::Vector3d>& p,
    const std::vector<Link>& links)
{
    const int n = static_cast<int>(links.size());
    const Eigen::Vector3d Z_AXIS(0.0, 0.0, 1.0);

    std::vector<Eigen::Vector3d> w(n + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> wd(n + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> a(n + 1, Eigen::Vector3d::Zero());
    a[0] = -g;

    // Forward recursion
    for (int i = 1; i <= n; ++i) {
        const Eigen::Matrix3d& R_i = R[i - 1];
        const Eigen::Vector3d& p_i = p[i - 1];
        const Link& link = links[i - 1];

        if (link.joint_type == 'R') {
            w[i] = R_i * w[i - 1] + Z_AXIS * qd[i - 1];
            wd[i] = R_i * wd[i - 1]
                    + Z_AXIS * qdd[i - 1]
                    + w[i].cross(Z_AXIS * qd[i - 1]);
            a[i] = R_i * (a[i - 1]
                          + wd[i - 1].cross(p_i)
                          + w[i - 1].cross(w[i - 1].cross(p_i)));
        } else {
            w[i] = R_i * w[i - 1];
            wd[i] = R_i * wd[i - 1];
            a[i] = R_i * (a[i - 1]
                          + wd[i - 1].cross(p_i)
                          + w[i - 1].cross(w[i - 1].cross(p_i)))
                   + Z_AXIS * qdd[i - 1]
                   + 2.0 * w[i].cross(Z_AXIS * qd[i - 1]);
        }
    }

    // Per-link forces and moments
    std::vector<Eigen::Vector3d> F(n), N(n);
    for (int i = 0; i < n; ++i) {
        const Link& link = links[i];
        Eigen::Vector3d r_c = link.r_com;
        Eigen::Vector3d a_c = a[i + 1]
                              + wd[i + 1].cross(r_c)
                              + w[i + 1].cross(w[i + 1].cross(r_c));
        F[i] = link.m * a_c;
        N[i] = link.I * wd[i + 1]
               + w[i + 1].cross(link.I * w[i + 1]);
    }

    // Backward recursion
    Eigen::Vector3d f_next = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_next = Eigen::Vector3d::Zero();
    Eigen::VectorXd tau(n);
    tau.setZero();

    for (int i = n - 1; i >= 0; --i) {
        Eigen::Matrix3d R_ip1 = (i < n - 1) ? R[i] : Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_ip1 = (i < n - 1) ? p[i] : Eigen::Vector3d::Zero();
        Eigen::Vector3d r_c = links[i].r_com;

        Eigen::Vector3d f_i = R_ip1 * f_next + F[i];
        Eigen::Vector3d n_i = N[i]
                              + R_ip1 * n_next
                              + r_c.cross(F[i])
                              + p_ip1.cross(R_ip1 * f_next);

        if (links[i].joint_type == 'R')
            tau[i] = n_i.dot(Z_AXIS);
        else
            tau[i] = f_i.dot(Z_AXIS);

        f_next = f_i;
        n_next = n_i;
    }
    return tau;
}
      
