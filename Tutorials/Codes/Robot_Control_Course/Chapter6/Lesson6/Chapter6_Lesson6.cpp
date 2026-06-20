
#include <Eigen/Dense>

class CartesianImpedanceController {
public:
    CartesianImpedanceController()
    {
        // Tangential and normal gains
        k_t_ = 2000.0;
        d_t_ = 40.0;
        k_n_ = 800.0;
        d_n_ = 20.0;
    }

    // Inputs:
    // x: current Cartesian position (3x1)
    // xd: desired Cartesian position (3x1)
    // dx: current Cartesian velocity (3x1)
    // dxd: desired Cartesian velocity (3x1)
    // f_ext: measured external force (3x1)
    // n: unit normal of the contact surface (3x1)
    // J: Jacobian matrix at current configuration (6xn)
    //
    // Output:
    // tau_cmd: desired joint torques (nx1)
    Eigen::VectorXd computeTorque(
        const Eigen::Vector3d &x,
        const Eigen::Vector3d &xd,
        const Eigen::Vector3d &dx,
        const Eigen::Vector3d &dxd,
        const Eigen::Vector3d &f_ext,
        const Eigen::Vector3d &n,
        const Eigen::MatrixXd &J,
        const Eigen::VectorXd &tau_dyn)
    {
        // Build tangent basis T_t from n (simple heuristic)
        Eigen::Vector3d t1;
        if (std::fabs(n.z()) > 0.5) {
            t1 = Eigen::Vector3d(1.0, 0.0, -n.x() / n.z());
        } else {
            t1 = Eigen::Vector3d(0.0, 1.0, -n.y() / n.z());
        }
        t1.normalize();
        Eigen::Vector3d t2 = n.cross(t1);
        t2.normalize();

        Eigen::Matrix<double, 3, 2> T_t;
        T_t.col(0) = t1;
        T_t.col(1) = t2;

        // Errors
        Eigen::Vector3d e = x - xd;
        Eigen::Vector3d de = dx - dxd;

        double e_n = n.dot(e);
        double de_n = n.dot(de);
        Eigen::Vector2d e_t = T_t.transpose() * e;
        Eigen::Vector2d de_t = T_t.transpose() * de;

        // Project measured force
        double f_n = n.dot(f_ext);
        Eigen::Vector2d f_t = T_t.transpose() * f_ext;

        // Desired normal force (lab parameter)
        double f_n_des = 20.0;

        // Tangential impedance (position tracking on the surface)
        Eigen::Vector2d f_t_cmd = -k_t_ * e_t - d_t_ * de_t;

        // Normal impedance around desired force
        double e_f = f_n - f_n_des;
        double f_n_cmd = -k_n_ * e_n - d_n_ * de_n - alpha_f_ * e_f;

        // Assemble Cartesian force vector
        Eigen::Vector3d f_c =
            T_t * f_t_cmd + n * f_n_cmd;

        // Joint torques: tau = tau_dyn + J.transpose() * f_c (force part only)
        Eigen::VectorXd tau_cmd = tau_dyn;
        tau_cmd.head(J.cols()) += J.topRows(3).transpose() * f_c;
        return tau_cmd;
    }

    void setGains(double k_t, double d_t, double k_n, double d_n, double alpha_f)
    {
        k_t_ = k_t;
        d_t_ = d_t;
        k_n_ = k_n;
        d_n_ = d_n;
        alpha_f_ = alpha_f;
    }

private:
    double k_t_, d_t_;
    double k_n_, d_n_;
    double alpha_f_ = 5.0;  // force feedback gain
};
