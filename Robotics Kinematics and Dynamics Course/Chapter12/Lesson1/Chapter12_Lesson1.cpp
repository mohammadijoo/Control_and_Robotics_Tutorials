#include <vector>
#include <Eigen/Dense>

struct Link {
    Eigen::Matrix3d R;      // R_i^{i-1}
    Eigen::Vector3d p;      // p_i (in frame i-1)
    Eigen::Vector3d rc;     // r_ci (in frame i)
    char joint_type;        // 'R' or 'P'
};

struct ForwardKinematicsResult {
    std::vector<Eigen::Vector3d> omega;
    std::vector<Eigen::Vector3d> alpha;
    std::vector<Eigen::Vector3d> a;
    std::vector<Eigen::Vector3d> ac;
};

ForwardKinematicsResult forwardRecursion(
    const std::vector<Link>& links,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd,
    const Eigen::Vector3d& g = Eigen::Vector3d(0.0, 0.0, 9.81))
{
    const int n = static_cast<int>(links.size());
    Eigen::Vector3d z(0.0, 0.0, 1.0);

    ForwardKinematicsResult res;
    res.omega.assign(n, Eigen::Vector3d::Zero());
    res.alpha.assign(n, Eigen::Vector3d::Zero());
    res.a.assign(n, Eigen::Vector3d::Zero());
    res.ac.assign(n, Eigen::Vector3d::Zero());

    Eigen::Vector3d omega_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d alpha_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_prev = -g;  // base acceleration

    for (int i = 0; i < n; ++i) {
        const Link& link = links[i];
        const double qi = q(i);
        const double qdi = qd(i);
        const double qddi = qdd(i);

        Eigen::Vector3d omega_i, alpha_i, a_i;

        Eigen::Vector3d a_base =
            a_prev
            + alpha_prev.cross(link.p)
            + omega_prev.cross(omega_prev.cross(link.p));

        if (link.joint_type == 'R') {
            omega_i = link.R * omega_prev + z * qdi;
            alpha_i = link.R * alpha_prev
                      + z * qddi
                      + omega_i.cross(z * qdi);
            a_i = link.R * a_base;
        } else if (link.joint_type == 'P') {
            omega_i = link.R * omega_prev;
            alpha_i = link.R * alpha_prev;
            a_i = link.R * a_base
                  + z * qddi
                  + (2.0 * omega_i).cross(z * qdi);
        } else {
            throw std::runtime_error("joint_type must be 'R' or 'P'");
        }

        Eigen::Vector3d ac_i =
            a_i
            + alpha_i.cross(link.rc)
            + omega_i.cross(omega_i.cross(link.rc));

        res.omega[i] = omega_i;
        res.alpha[i] = alpha_i;
        res.a[i] = a_i;
        res.ac[i] = ac_i;

        omega_prev = omega_i;
        alpha_prev = alpha_i;
        a_prev = a_i;
    }

    return res;
}
      
