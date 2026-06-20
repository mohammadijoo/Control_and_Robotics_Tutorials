
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct RobotEKF {
    int n;          // number of joints
    double dt;      // sampling time
    VectorXd x;     // state [q; dq]
    MatrixXd P;     // covariance
    MatrixXd Q;     // process noise
    MatrixXd R;     // measurement noise

    RobotEKF(int n_dof, double dt_)
        : n(n_dof), dt(dt_)
    {
        int dim_x = 2 * n;
        x = VectorXd::Zero(dim_x);
        P = MatrixXd::Identity(dim_x, dim_x) * 1e-2;

        Q = MatrixXd::Zero(dim_x, dim_x);
        Q.block(0, 0, n, n) = 1e-6 * MatrixXd::Identity(n, n);
        Q.block(n, n, n, n) = 1e-4 * MatrixXd::Identity(n, n);

        int dim_z = n + 3;  // q encoders + 3D ee position
        R = MatrixXd::Zero(dim_z, dim_z);
        R.block(0, 0, n, n) = 1e-5 * MatrixXd::Identity(n, n);
        R.block(n, n, 3, 3) = 1e-4 * MatrixXd::Identity(3, 3);
    }

    VectorXd f(const VectorXd& xk, const VectorXd& u) const {
        int dim_x = xk.size();
        int n_local = n;
        VectorXd q = xk.segment(0, n_local);
        VectorXd dq = xk.segment(n_local, n_local);

        VectorXd qddot = robot_forward_dynamics(q, dq, u); // user-defined

        VectorXd x_next(dim_x);
        x_next.segment(0, n_local) = q + dt * dq;
        x_next.segment(n_local, n_local) = dq + dt * qddot;
        return x_next;
    }

    MatrixXd F_jacobian(const VectorXd& xk, const VectorXd& u) const {
        int dim_x = xk.size();
        MatrixXd F(dim_x, dim_x);
        VectorXd fx = f(xk, u);
        double eps = 1e-6;

        for (int i = 0; i < dim_x; ++i) {
            VectorXd dx = VectorXd::Zero(dim_x);
            dx(i) = eps;
            VectorXd f_plus = f(xk + dx, u);
            F.col(i) = (f_plus - fx) / eps;
        }
        return F;
    }

    VectorXd h(const VectorXd& xk) const {
        int dim_x = xk.size();
        int n_local = n;
        VectorXd q = xk.segment(0, n_local);
        VectorXd z(dim_z());
        // encoders
        z.segment(0, n_local) = q;
        // ee position
        VectorXd ee = forward_kinematics(q);  // size 3
        z.segment(n_local, 3) = ee;
        return z;
    }

    MatrixXd H_jacobian(const VectorXd& xk) const {
        int dim_x = xk.size();
        int dimz = dim_z();
        MatrixXd H(dimz, dim_x);
        double eps = 1e-6;
        VectorXd h0 = h(xk);

        for (int i = 0; i < dim_x; ++i) {
            VectorXd dx = VectorXd::Zero(dim_x);
            dx(i) = eps;
            VectorXd h_plus = h(xk + dx);
            H.col(i) = (h_plus - h0) / eps;
        }
        return H;
    }

    int dim_z() const { return n + 3; }

    void predict(const VectorXd& u) {
        VectorXd x_prev = x;
        MatrixXd P_prev = P;

        x = f(x_prev, u);
        MatrixXd F = F_jacobian(x_prev, u);
        P = F * P_prev * F.transpose() + Q;
    }

    void update(const VectorXd& z_meas) {
        VectorXd x_pred = x;
        MatrixXd P_pred = P;

        VectorXd z_pred = h(x_pred);
        MatrixXd H = H_jacobian(x_pred);
        VectorXd y = z_meas - z_pred;

        MatrixXd S = H * P_pred * H.transpose() + R;
        MatrixXd K = P_pred * H.transpose() * S.inverse();

        x = x_pred + K * y;
        MatrixXd I = MatrixXd::Identity(P_pred.rows(), P_pred.cols());
        P = (I - K * H) * P_pred * (I - K * H).transpose() + K * R * K.transpose();
    }
};

// Placeholders for robot functions.
VectorXd robot_forward_dynamics(const VectorXd& q,
                                const VectorXd& dq,
                                const VectorXd& tau)
{
    // TODO: implement using your robot dynamics library
    return tau; // qddot = tau (dummy)
}

VectorXd forward_kinematics(const VectorXd& q)
{
    // TODO: implement your forward kinematics
    VectorXd ee(3);
    ee.setZero();
    ee(0) = q.sum(); // dummy: x = sum of joint angles
    return ee;
}
