/*
Chapter 11 - Lesson 5: Lab: EKF-SLAM in a Small World
Autonomous Mobile Robots (Control Engineering)

Minimal C++11 EKF-SLAM (joint-state) demo using Eigen.
This is a compact educational implementation intended for small landmark counts.

Build (example):
  g++ -O2 -std=c++11 Chapter11_Lesson5.cpp -I /path/to/eigen -o ekf_slam

Note: For brevity, this file focuses on the core math and a small simulation loop.
*/

#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include "Eigen/Dense"

static double wrap_pi(double a) {
    const double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2.0 * pi);
    if (a < 0.0) a += 2.0 * pi;
    return a - pi;
}

struct Obs {
    int id;
    Eigen::Vector2d z; // [range, bearing]
};

struct EKFSLAM {
    Eigen::VectorXd mu;   // [x,y,theta, l1x,l1y, ...]
    Eigen::MatrixXd P;
    Eigen::Matrix2d Q;    // control noise (v,w)
    Eigen::Matrix2d R;    // meas noise (r,b)
    std::map<int,int> seen; // id -> index
    int nL;

    EKFSLAM(const Eigen::Matrix2d& Q_, const Eigen::Matrix2d& R_)
        : Q(Q_), R(R_), nL(0) {
        mu = Eigen::VectorXd::Zero(3);
        P  = Eigen::MatrixXd::Identity(3,3) * 1e-6;
    }

    Eigen::Vector3d motion(const Eigen::Vector3d& xr, const Eigen::Vector2d& u, double dt) {
        double x = xr(0), y = xr(1), th = xr(2);
        double v = u(0), w = u(1);
        Eigen::Vector3d out;
        out(0) = x + v * dt * std::cos(th);
        out(1) = y + v * dt * std::sin(th);
        out(2) = wrap_pi(th + w * dt);
        return out;
    }

    Eigen::Matrix3d F_jac(const Eigen::Vector3d& xr, const Eigen::Vector2d& u, double dt) {
        double th = xr(2);
        double v  = u(0);
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0,2) = -v * dt * std::sin(th);
        F(1,2) =  v * dt * std::cos(th);
        return F;
    }

    Eigen::Matrix<double,3,2> L_jac(const Eigen::Vector3d& xr, double dt) {
        double th = xr(2);
        Eigen::Matrix<double,3,2> L;
        L.setZero();
        L(0,0) = dt * std::cos(th);
        L(1,0) = dt * std::sin(th);
        L(2,1) = dt;
        return L;
    }

    Eigen::Vector2d meas(const Eigen::Vector3d& xr, const Eigen::Vector2d& lm) {
        double dx = lm(0) - xr(0);
        double dy = lm(1) - xr(1);
        double r  = std::sqrt(dx*dx + dy*dy);
        double b  = wrap_pi(std::atan2(dy, dx) - xr(2));
        Eigen::Vector2d z;
        z << r, b;
        return z;
    }

    void predict(const Eigen::Vector2d& u, double dt) {
        Eigen::Vector3d xr = mu.segment<3>(0);
        Eigen::Vector3d xr2 = motion(xr, u, dt);
        Eigen::Matrix3d F = F_jac(xr, u, dt);
        Eigen::Matrix<double,3,2> L = L_jac(xr, dt);
        Eigen::Matrix3d Qx = L * Q * L.transpose();

        int dim = 3 + 2*nL;
        Eigen::MatrixXd Fbig = Eigen::MatrixXd::Identity(dim, dim);
        Fbig.block<3,3>(0,0) = F;

        mu.segment<3>(0) = xr2;
        P = Fbig * P * Fbig.transpose();
        P.block<3,3>(0,0) += Qx;
        P = 0.5*(P + P.transpose());
    }

    void augment_landmark(int id, const Eigen::Vector2d& z) {
        Eigen::Vector3d xr = mu.segment<3>(0);
        double r = z(0), b = z(1);
        double th = xr(2);
        Eigen::Vector2d lm;
        lm(0) = xr(0) + r * std::cos(th + b);
        lm(1) = xr(1) + r * std::sin(th + b);

        int old_dim = mu.size();
        Eigen::VectorXd mu2(old_dim + 2);
        mu2.head(old_dim) = mu;
        mu2.tail<2>() = lm;
        mu = mu2;

        // Jacobians Gx (2x3) and Gz (2x2)
        double c = std::cos(th + b);
        double s = std::sin(th + b);
        Eigen::Matrix<double,2,3> Gx;
        Gx << 1.0, 0.0, -r*s,
              0.0, 1.0,  r*c;
        Eigen::Matrix2d Gz;
        Gz << c, -r*s,
              s,  r*c;

        Eigen::Matrix3d Prr = P.block<3,3>(0,0);
        Eigen::Matrix2d Pmm = Gx * Prr * Gx.transpose() + Gz * R * Gz.transpose();

        Eigen::MatrixXd P2 = Eigen::MatrixXd::Zero(old_dim + 2, old_dim + 2);
        P2.block(0,0,old_dim,old_dim) = P;
        Eigen::MatrixXd Px_r = P.block(0,0,old_dim,3);
        Eigen::MatrixXd Pxm = Px_r * Gx.transpose(); // old_dim x 2
        P2.block(0, old_dim, old_dim, 2) = Pxm;
        P2.block(old_dim, 0, 2, old_dim) = Pxm.transpose();
        P2.block(old_dim, old_dim, 2, 2) = Pmm;
        P = 0.5*(P2 + P2.transpose());

        seen[id] = nL;
        nL += 1;
    }

    void update_one(int id, const Eigen::Vector2d& z) {
        if (seen.find(id) == seen.end()) {
            augment_landmark(id, z);
            return;
        }
        int idx = seen[id];
        Eigen::Vector3d xr = mu.segment<3>(0);
        Eigen::Vector2d lm = mu.segment<2>(3 + 2*idx);

        Eigen::Vector2d zhat = meas(xr, lm);
        Eigen::Vector2d y;
        y(0) = z(0) - zhat(0);
        y(1) = wrap_pi(z(1) - zhat(1));

        double dx = lm(0) - xr(0);
        double dy = lm(1) - xr(1);
        double q = dx*dx + dy*dy;
        double r = std::sqrt(std::max(q, 1e-12));

        Eigen::Matrix<double,2,3> Hxr;
        Hxr << -dx/r, -dy/r, 0.0,
                dy/q, -dx/q, -1.0;
        Eigen::Matrix2d Hlm;
        Hlm << dx/r, dy/r,
              -dy/q, dx/q;

        int dim = 3 + 2*nL;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, dim);
        H.block<2,3>(0,0) = Hxr;
        H.block<2,2>(0, 3 + 2*idx) = Hlm;

        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix2d Sinv = S.inverse();
        double d2 = y.transpose() * Sinv * y;
        if (d2 > 9.210) { // 99% chi2 gate, df=2
            return;
        }

        Eigen::MatrixXd K = P * H.transpose() * Sinv;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);

        mu = mu + K * y;
        mu(2) = wrap_pi(mu(2));
        P = (I - K*H) * P * (I - K*H).transpose() + K*R*K.transpose();
        P = 0.5*(P + P.transpose());
    }
};

int main() {
    // Small world landmarks
    std::map<int, Eigen::Vector2d> landmarks;
    landmarks[1] = Eigen::Vector2d(4.0, 3.5);
    landmarks[2] = Eigen::Vector2d(8.5, 1.5);
    landmarks[3] = Eigen::Vector2d(7.5, 7.5);

    double dt = 0.1;
    int T = 120;

    // Noise
    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0,0) = 0.05*0.05; // v
    Q(1,1) = 0.03*0.03; // w
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = 0.12*0.12; // range
    R(1,1) = (2.5*M_PI/180.0)*(2.5*M_PI/180.0); // bearing

    EKFSLAM ekf(Q, R);
    ekf.P = Eigen::MatrixXd::Zero(3,3);
    ekf.P(0,0) = 0.05*0.05;
    ekf.P(1,1) = 0.05*0.05;
    ekf.P(2,2) = (5.0*M_PI/180.0)*(5.0*M_PI/180.0);

    // Ground truth
    Eigen::Vector3d xgt(1.0, 1.0, 20.0*M_PI/180.0);

    for (int k = 0; k != T; ++k) {
        double v = 0.7 + 0.15 * std::sin(0.08 * k);
        double w = 0.35 * std::sin(0.05 * k);
        Eigen::Vector2d u(v, w);

        // propagate truth
        xgt = ekf.motion(xgt, u, dt);

        // EKF predict
        ekf.predict(u, dt);

        // Generate perfect ID observations with noise-free gating (for demo)
        for (auto const& it : landmarks) {
            int id = it.first;
            Eigen::Vector2d lm = it.second;
            Eigen::Vector2d ztrue = ekf.meas(xgt, lm);
            if (ztrue(0) < 6.0 && std::abs(ztrue(1)) < (70.0*M_PI/180.0)) {
                // add noise
                Eigen::Vector2d z = ztrue;
                // simple deterministic pseudo-noise (avoid RNG to keep file minimal)
                z(0) += 0.02 * std::sin(0.31*k + id);
                z(1)  = wrap_pi(z(1) + (1.0*M_PI/180.0) * std::sin(0.19*k + 2*id));
                ekf.update_one(id, z);
            }
        }
    }

    std::cout << "Final estimate [x y theta]: " << ekf.mu.segment<3>(0).transpose() << std::endl;
    std::cout << "Final ground truth [x y theta]: " << xgt.transpose() << std::endl;

    for (auto const& it : ekf.seen) {
        int id = it.first;
        int idx = it.second;
        Eigen::Vector2d lm_est = ekf.mu.segment<2>(3 + 2*idx);
        std::cout << "Landmark " << id << " est: " << lm_est.transpose() << std::endl;
    }

    return 0;
}
