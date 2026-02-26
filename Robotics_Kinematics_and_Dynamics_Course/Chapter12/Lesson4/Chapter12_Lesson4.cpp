#include <Eigen/Dense>
#include <vector>

using Eigen::Vector3d;
using Eigen::Matrix3d;

struct Link {
    Matrix3d R;      // R_i^{i-1}
    Vector3d p;      // p_i^{i-1} in frame i-1
    Vector3d r_c;    // CoM offset in frame i
    double m;
    Matrix3d I;      // inertia about frame i origin
    char joint_type; // 'R' or 'P'
};

std::vector<double> newtonEuler(
    const std::vector<Link>& links,
    const std::vector<double>& q,
    const std::vector<double>& qd,
    const std::vector<double>& qdd,
    const Vector3d& g0
){
    const int n = static_cast<int>(links.size());
    const Vector3d z_axis(0.0, 0.0, 1.0);

    std::vector<Vector3d> omega(n+1, Vector3d::Zero());
    std::vector<Vector3d> omegadot(n+1, Vector3d::Zero());
    std::vector<Vector3d> a(n+1, Vector3d::Zero());
    std::vector<Vector3d> a_c(n, Vector3d::Zero());

    // base quantities
    omega[0].setZero();
    omegadot[0].setZero();
    a[0] = -g0;

    // forward recursion
    for (int i = 1; i <= n; ++i) {
        const Link& L = links[i-1];
        const Matrix3d& R = L.R;
        const Vector3d& p = L.p;
        char jt = L.joint_type;

        if (jt == 'R') {
            omega[i] = R * omega[i-1] + z_axis * qd[i-1];
            omegadot[i] = R * omegadot[i-1]
                        + z_axis * qdd[i-1]
                        + omega[i].cross(z_axis * qd[i-1]);
            a[i] = R * ( a[i-1]
                   + omegadot[i-1].cross(p)
                   + omega[i-1].cross(omega[i-1].cross(p)) );
        } else { // prismatic
            omega[i] = R * omega[i-1];
            omegadot[i] = R * omegadot[i-1];
            a[i] = R * ( a[i-1]
                   + omegadot[i-1].cross(p)
                   + omega[i-1].cross(omega[i-1].cross(p)) )
                 + 2.0 * omega[i].cross(z_axis * qd[i-1])
                 + z_axis * qdd[i-1];
        }

        a_c[i-1] = a[i]
                 + omegadot[i].cross(L.r_c)
                 + omega[i].cross(omega[i].cross(L.r_c));
    }

    // backward recursion
    std::vector<Vector3d> f(n+2, Vector3d::Zero());
    std::vector<Vector3d> nvec(n+2, Vector3d::Zero());
    std::vector<double> tau(n, 0.0);

    for (int i = n; i >= 1; --i) {
        const Link& L = links[i-1];

        Matrix3d R_next = Matrix3d::Identity();
        Vector3d p_next = Vector3d::Zero();
        if (i < n) {
            R_next = links[i].R.transpose(); // R_i^{i+1}
            p_next = links[i].p;             // p_{i+1}^{i} approx
        }

        f[i] = L.m * a_c[i-1] + R_next * f[i+1];
        nvec[i] = L.I * omegadot[i]
                  + omega[i].cross(L.I * omega[i])
                  + L.r_c.cross(L.m * a_c[i-1])
                  + R_next * nvec[i+1]
                  + (p_next + L.r_c).cross(R_next * f[i+1]);

        if (L.joint_type == 'R') {
            tau[i-1] = nvec[i].dot(z_axis);
        } else {
            tau[i-1] = f[i].dot(z_axis);
        }
    }

    return tau;
}
      
