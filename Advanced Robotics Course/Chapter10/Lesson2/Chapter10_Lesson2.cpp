#include <Eigen/Dense>

struct KalmanFilterCV2D {
    double dt;
    Eigen::Matrix4d F;
    Eigen::Matrix<double, 2, 4> H;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
    Eigen::Vector4d x;
    Eigen::Matrix4d P;

    KalmanFilterCV2D(double dt_, double process_var, double meas_var)
        : dt(dt_)
    {
        F.setIdentity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        H.setZero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;

        double q = process_var;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt2 * dt2;

        Eigen::Matrix2d Q1d;
        Q1d << 0.25 * dt4, 0.5 * dt3,
                0.5 * dt3,  dt2;

        Q.setZero();
        Q.block<2, 2>(0, 0) = Q1d;
        Q.block<2, 2>(2, 2) = Q1d;
        Q *= q;

        R = meas_var * Eigen::Matrix2d::Identity();

        x.setZero();
        P = Eigen::Matrix4d::Identity() * 1e3;
    }

    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::Vector2d& z) {
        Eigen::Vector2d y = z - H * x;
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
        x = x + K * y;
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        P = (I - K * H) * P;
    }

    Eigen::Vector2d position() const {
        return x.segment<2>(0);
    }
};
      
