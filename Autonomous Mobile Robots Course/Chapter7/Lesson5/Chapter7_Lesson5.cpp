// Chapter7_Lesson5.cpp
// EKF/UKF localization lab: Wheel + IMU + GPS (2D ground robot)
// Dependencies: Eigen (header-only for basic usage)
// Build (example):
//   g++ -O2 -std=c++17 Chapter7_Lesson5.cpp -I /usr/include/eigen3 -o ch7_l5
//
// Note: This file is self-contained for teaching. For production, prefer
// ROS2 robot_localization (EKF/UKF) and properly time-synchronized sensor inputs.

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static double wrap_angle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

struct SimData {
    std::vector<double> t;
    std::vector<VectorXd> Xtrue;   // 6D state
    std::vector<double> imu_w;     // omega_m
    std::vector<double> imu_a;     // a_m
    std::vector<int> wheel_idx;
    std::vector<double> wheel_v;
    std::vector<double> wheel_w;
    std::vector<int> gps_idx;
    std::vector<Eigen::Vector2d> gps_xy;
};

static SimData simulate(double T, double dt_imu, double dt_wheel, double dt_gps) {
    int n = static_cast<int>(T / dt_imu) + 1;
    SimData d;
    d.t.resize(n);
    d.Xtrue.resize(n);
    d.imu_w.resize(n);
    d.imu_a.resize(n);

    std::mt19937 rng(7);
    std::normal_distribution<double> N01(0.0, 1.0);

    // Noise parameters
    double sigma_g = 0.02, sigma_a = 0.20;
    double sigma_bg_rw = 5e-4, sigma_ba_rw = 2e-3;
    double sigma_vw = 0.10, sigma_ww = 0.02;
    double sigma_gps = 1.5;

    // Initial truth: x,y,theta,v,bg,ba
    d.Xtrue[0] = VectorXd(6);
    d.Xtrue[0] << 0.0, 0.0, 0.2, 1.0, 0.02, -0.05;

    for (int k = 0; k < n; ++k) d.t[k] = k * dt_imu;

    // Generate truth
    for (int k = 0; k < n - 1; ++k) {
        double tt = d.t[k];
        double omega_cmd = 0.25 * std::sin(0.2 * tt) + 0.05 * std::sin(1.1 * tt);
        double a_cmd = 0.4 * std::sin(0.1 * tt);

        auto xk = d.Xtrue[k];
        double x = xk(0), y = xk(1), th = xk(2), v = xk(3), bg = xk(4), ba = xk(5);

        double bg_next = bg + sigma_bg_rw * std::sqrt(dt_imu) * N01(rng);
        double ba_next = ba + sigma_ba_rw * std::sqrt(dt_imu) * N01(rng);

        double th_next = wrap_angle(th + omega_cmd * dt_imu);
        double v_next  = v + a_cmd * dt_imu;
        double x_next  = x + v * dt_imu * std::cos(th);
        double y_next  = y + v * dt_imu * std::sin(th);

        d.Xtrue[k + 1] = VectorXd(6);
        d.Xtrue[k + 1] << x_next, y_next, th_next, v_next, bg_next, ba_next;

        // IMU measurements
        d.imu_w[k] = omega_cmd + bg + sigma_g * N01(rng);
        d.imu_a[k] = a_cmd + ba + sigma_a * N01(rng);
    }
    d.imu_w[n - 1] = d.imu_w[n - 2];
    d.imu_a[n - 1] = d.imu_a[n - 2];

    // Wheel and GPS sampling indices
    int step_wheel = std::max(1, static_cast<int>(std::round(dt_wheel / dt_imu)));
    int step_gps   = std::max(1, static_cast<int>(std::round(dt_gps / dt_imu)));

    for (int k = 0; k < n; k += step_wheel) {
        d.wheel_idx.push_back(k);
        d.wheel_v.push_back(d.Xtrue[k](3) + sigma_vw * N01(rng));
        // Wheel yaw rate measures true omega_cmd (approx) with noise; we can reconstruct from truth by finite diff,
        // but for teaching we reuse the IMU-free omega profile encoded in imu_w - bg.
        d.wheel_w.push_back((d.imu_w[k] - d.Xtrue[k](4)) + sigma_ww * N01(rng));
    }
    for (int k = 0; k < n; k += step_gps) {
        d.gps_idx.push_back(k);
        Eigen::Vector2d z;
        z << d.Xtrue[k](0) + sigma_gps * N01(rng), d.Xtrue[k](1) + sigma_gps * N01(rng);
        d.gps_xy.push_back(z);
    }

    return d;
}

struct EKF {
    VectorXd x;  // 6
    MatrixXd P;  // 6x6
    MatrixXd Qc; // 4x4 continuous: [n_g, n_a, w_bg, w_ba]
    MatrixXd Rw; // 2x2
    MatrixXd Rg; // 2x2

    EKF(const VectorXd& x0, const MatrixXd& P0, const MatrixXd& Qc_, const MatrixXd& Rw_, const MatrixXd& Rg_)
        : x(x0), P(P0), Qc(Qc_), Rw(Rw_), Rg(Rg_) {}

    void predict(double omega_m, double a_m, double dt) {
        double X = x(0), Y = x(1), th = x(2), v = x(3), bg = x(4), ba = x(5);
        double omega = omega_m - bg;
        double acc = a_m - ba;

        VectorXd xp(6);
        xp(0) = X + v * dt * std::cos(th);
        xp(1) = Y + v * dt * std::sin(th);
        xp(2) = wrap_angle(th + omega * dt);
        xp(3) = v + acc * dt;
        xp(4) = bg;
        xp(5) = ba;
        x = xp;

        MatrixXd F = MatrixXd::Identity(6, 6);
        F(0,2) = -v * dt * std::sin(th);
        F(0,3) =  dt * std::cos(th);
        F(1,2) =  v * dt * std::cos(th);
        F(1,3) =  dt * std::sin(th);
        F(2,4) = -dt;
        F(3,5) = -dt;

        MatrixXd G = MatrixXd::Zero(6, 4);
        G(2,0) = dt;
        G(3,1) = dt;
        G(4,2) = std::sqrt(dt);
        G(5,3) = std::sqrt(dt);

        MatrixXd Qd = G * Qc * G.transpose();
        P = F * P * F.transpose() + Qd;
    }

    void update_wheel(double v_w, double omega_w, double omega_m) {
        VectorXd z(2), h(2);
        z << v_w, omega_w;
        h << x(3), (omega_m - x(4));

        MatrixXd H = MatrixXd::Zero(2, 6);
        H(0,3) = 1.0;
        H(1,4) = -1.0;

        update(z, h, H, Rw);
    }

    void update_gps(const Eigen::Vector2d& zxy) {
        VectorXd z(2), h(2);
        z << zxy(0), zxy(1);
        h << x(0), x(1);

        MatrixXd H = MatrixXd::Zero(2, 6);
        H(0,0) = 1.0;
        H(1,1) = 1.0;

        update(z, h, H, Rg);
    }

    void update(const VectorXd& z, const VectorXd& h, const MatrixXd& H, const MatrixXd& R) {
        VectorXd y = z - h;
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * y;
        x(2) = wrap_angle(x(2));

        MatrixXd I = MatrixXd::Identity(6, 6);
        P = (I - K*H) * P * (I - K*H).transpose() + K * R * K.transpose();
    }
};

struct UKF {
    VectorXd x;
    MatrixXd P;
    MatrixXd Qd;
    MatrixXd Rw, Rg;

    int n;
    double alpha, beta, kappa, lambda;
    VectorXd Wm, Wc;

    UKF(const VectorXd& x0, const MatrixXd& P0, const MatrixXd& Qd_, const MatrixXd& Rw_, const MatrixXd& Rg_,
        double alpha_=1e-3, double beta_=2.0, double kappa_=0.0)
        : x(x0), P(P0), Qd(Qd_), Rw(Rw_), Rg(Rg_) {
        n = 6;
        alpha = alpha_;
        beta = beta_;
        kappa = kappa_;
        lambda = alpha*alpha*(n + kappa) - n;
        Wm = VectorXd::Constant(2*n + 1, 1.0 / (2.0*(n + lambda)));
        Wc = Wm;
        Wm(0) = lambda / (n + lambda);
        Wc(0) = Wm(0) + (1.0 - alpha*alpha + beta);
    }

    MatrixXd sigma_points(const VectorXd& x0, const MatrixXd& P0) const {
        Eigen::LLT<MatrixXd> llt((n + lambda) * P0);
        MatrixXd S = llt.matrixL();
        MatrixXd X(2*n + 1, n);
        X.row(0) = x0.transpose();
        for (int i = 0; i < n; ++i) {
            X.row(1 + i)     = (x0 + S.col(i)).transpose();
            X.row(1 + i + n) = (x0 - S.col(i)).transpose();
        }
        // wrap theta
        for (int i = 0; i < X.rows(); ++i) X(i,2) = wrap_angle(X(i,2));
        return X;
    }

    VectorXd f(const VectorXd& s, double omega_m, double a_m, double dt) const {
        double X = s(0), Y = s(1), th = s(2), v = s(3), bg = s(4), ba = s(5);
        double omega = omega_m - bg;
        double acc = a_m - ba;
        VectorXd xp(6);
        xp(0) = X + v * dt * std::cos(th);
        xp(1) = Y + v * dt * std::sin(th);
        xp(2) = wrap_angle(th + omega * dt);
        xp(3) = v + acc * dt;
        xp(4) = bg;
        xp(5) = ba;
        return xp;
    }

    static double angle_mean(const VectorXd& angles, const VectorXd& w) {
        double s = 0.0, c = 0.0;
        for (int i = 0; i < angles.size(); ++i) {
            s += w(i) * std::sin(angles(i));
            c += w(i) * std::cos(angles(i));
        }
        return std::atan2(s, c);
    }

    void predict(double omega_m, double a_m, double dt) {
        MatrixXd X = sigma_points(x, P);
        MatrixXd Xp(2*n + 1, n);
        for (int i = 0; i < X.rows(); ++i) Xp.row(i) = f(X.row(i).transpose(), omega_m, a_m, dt).transpose();

        VectorXd xmean = VectorXd::Zero(n);
        for (int i = 0; i < Xp.rows(); ++i) xmean += Wm(i) * Xp.row(i).transpose();
        // circular mean for theta
        xmean(2) = angle_mean(Xp.col(2), Wm);

        MatrixXd Pp = MatrixXd::Zero(n, n);
        for (int i = 0; i < Xp.rows(); ++i) {
            VectorXd dx = Xp.row(i).transpose() - xmean;
            dx(2) = wrap_angle(dx(2));
            Pp += Wc(i) * (dx * dx.transpose());
        }
        Pp += Qd;

        x = xmean;
        P = Pp;
    }

    void update_wheel(double v_w, double omega_w, double omega_m) {
        VectorXd z(2);
        z << v_w, omega_w;
        auto h = [&](const VectorXd& s) {
            VectorXd hz(2);
            hz << s(3), (omega_m - s(4));
            return hz;
        };
        update(z, h, Rw);
    }

    void update_gps(const Eigen::Vector2d& zxy) {
        VectorXd z(2);
        z << zxy(0), zxy(1);
        auto h = [&](const VectorXd& s) {
            VectorXd hz(2);
            hz << s(0), s(1);
            return hz;
        };
        update(z, h, Rg);
    }

    template <typename HFun>
    void update(const VectorXd& z, HFun h, const MatrixXd& R) {
        MatrixXd X = sigma_points(x, P);
        int m = static_cast<int>(z.size());
        MatrixXd Z(2*n + 1, m);
        for (int i = 0; i < X.rows(); ++i) Z.row(i) = h(X.row(i).transpose()).transpose();

        VectorXd zmean = VectorXd::Zero(m);
        for (int i = 0; i < Z.rows(); ++i) zmean += Wm(i) * Z.row(i).transpose();

        MatrixXd S = MatrixXd::Zero(m, m);
        MatrixXd Pxz = MatrixXd::Zero(n, m);
        for (int i = 0; i < Z.rows(); ++i) {
            VectorXd dz = Z.row(i).transpose() - zmean;
            VectorXd dx = X.row(i).transpose() - x;
            dx(2) = wrap_angle(dx(2));
            S += Wc(i) * (dz * dz.transpose());
            Pxz += Wc(i) * (dx * dz.transpose());
        }
        S += R;

        MatrixXd K = Pxz * S.inverse();
        VectorXd innov = z - zmean;
        x = x + K * innov;
        x(2) = wrap_angle(x(2));
        P = P - K * S * K.transpose();
    }
};

int main() {
    double T = 80.0;
    double dt_imu = 0.01, dt_wheel = 0.05, dt_gps = 1.0;

    SimData d = simulate(T, dt_imu, dt_wheel, dt_gps);

    VectorXd x0(6);
    x0 << 0.5, -1.0, 0.0, 0.5, 0.0, 0.0;
    MatrixXd P0 = MatrixXd::Zero(6,6);
    P0.diagonal() << 4.0, 4.0, std::pow(20.0*M_PI/180.0, 2), 1.0, std::pow(0.05,2), std::pow(0.2,2);

    // Qc for EKF (gyro, accel, bg_rw, ba_rw)
    double sigma_g = 0.02, sigma_a = 0.20;
    double sigma_bg_rw = 5e-4, sigma_ba_rw = 2e-3;
    MatrixXd Qc = MatrixXd::Zero(4,4);
    Qc.diagonal() << sigma_g*sigma_g, sigma_a*sigma_a, sigma_bg_rw*sigma_bg_rw, sigma_ba_rw*sigma_ba_rw;

    double sigma_vw = 0.10, sigma_ww = 0.02, sigma_gps = 1.5;
    MatrixXd Rw = MatrixXd::Zero(2,2);
    Rw.diagonal() << sigma_vw*sigma_vw, sigma_ww*sigma_ww;
    MatrixXd Rg = MatrixXd::Zero(2,2);
    Rg.diagonal() << sigma_gps*sigma_gps, sigma_gps*sigma_gps;

    EKF ekf(x0, P0, Qc, Rw, Rg);

    MatrixXd Qd = MatrixXd::Zero(6,6);
    Qd.diagonal() << 0.0, 0.0, std::pow(sigma_g*dt_imu, 2), std::pow(sigma_a*dt_imu,2),
                     std::pow(sigma_bg_rw*std::sqrt(dt_imu),2), std::pow(sigma_ba_rw*std::sqrt(dt_imu),2);
    UKF ukf(x0, P0, Qd, Rw, Rg);

    // Index maps
    std::vector<int> wheel_map(d.Xtrue.size(), -1);
    for (int i = 0; i < (int)d.wheel_idx.size(); ++i) wheel_map[d.wheel_idx[i]] = i;
    std::vector<int> gps_map(d.Xtrue.size(), -1);
    for (int i = 0; i < (int)d.gps_idx.size(); ++i) gps_map[d.gps_idx[i]] = i;

    VectorXd err_sum_ekf = VectorXd::Zero(3);
    VectorXd err_sum_ukf = VectorXd::Zero(3);

    for (int k = 0; k < (int)d.t.size(); ++k) {
        double omega_m = d.imu_w[k];
        double a_m = d.imu_a[k];

        ekf.predict(omega_m, a_m, dt_imu);
        ukf.predict(omega_m, a_m, dt_imu);

        if (wheel_map[k] >= 0) {
            int j = wheel_map[k];
            ekf.update_wheel(d.wheel_v[j], d.wheel_w[j], omega_m);
            ukf.update_wheel(d.wheel_v[j], d.wheel_w[j], omega_m);
        }
        if (gps_map[k] >= 0) {
            int j = gps_map[k];
            ekf.update_gps(d.gps_xy[j]);
            ukf.update_gps(d.gps_xy[j]);
        }

        VectorXd xt = d.Xtrue[k];
        VectorXd de = ekf.x - xt;
        de(2) = wrap_angle(de(2));
        VectorXd du = ukf.x - xt;
        du(2) = wrap_angle(du(2));

        err_sum_ekf(0) += de(0)*de(0) + de(1)*de(1);
        err_sum_ekf(1) += de(2)*de(2);
        err_sum_ukf(0) += du(0)*du(0) + du(1)*du(1);
        err_sum_ukf(1) += du(2)*du(2);
    }

    double N = (double)d.t.size();
    std::cout << "Position RMSE (m): EKF=" << std::sqrt(err_sum_ekf(0)/N) << " UKF=" << std::sqrt(err_sum_ukf(0)/N) << "\n";
    std::cout << "Heading RMSE (rad): EKF=" << std::sqrt(err_sum_ekf(1)/N) << " UKF=" << std::sqrt(err_sum_ukf(1)/N) << "\n";
    return 0;
}
