// Chapter 11 - SLAM I (Filter-Based SLAM)
// Lesson 2: EKF-SLAM (structure and limitations)
//
// Minimal educational EKF-SLAM skeleton using Eigen for linear algebra.
// Assumptions:
// - 2D robot pose (x,y,theta)
// - Range-bearing measurements to point landmarks
// - Known data association (landmark id given for each measurement)
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter11_Lesson2.cpp -I /usr/include/eigen3 -o ekf_slam
//
// Note: This is intentionally compact and focuses on structure, not production robustness.

#include <iostream>
#include <unordered_map>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

static double WrapAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

struct Measurement {
    int id;
    double range;
    double bearing; // radians
};

class EKFSLAM {
public:
    EKFSLAM() {
        mu_ = Eigen::VectorXd::Zero(3);
        P_  = Eigen::MatrixXd::Identity(3,3) * 1e-6;
    }

    int NumLandmarks() const { return (int)id_to_slot_.size(); }
    int Dim() const { return 3 + 2 * NumLandmarks(); }

    void Predict(double v, double w, double dt, const Eigen::Matrix2d& Q) {
        double x = mu_(0), y = mu_(1), th = mu_(2);

        double x_new, y_new, th_new;
        if (std::abs(w) < 1e-9) {
            x_new = x + v * dt * std::cos(th);
            y_new = y + v * dt * std::sin(th);
            th_new = th;
        } else {
            x_new = x + (v / w) * (std::sin(th + w*dt) - std::sin(th));
            y_new = y - (v / w) * (std::cos(th + w*dt) - std::cos(th));
            th_new = th + w * dt;
        }
        th_new = WrapAngle(th_new);

        mu_(0) = x_new; mu_(1) = y_new; mu_(2) = th_new;

        const int D = Dim();
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(D, D);
        Eigen::MatrixXd Gu = Eigen::MatrixXd::Zero(D, 2);

        if (std::abs(w) < 1e-9) {
            F(0,2) = -v * dt * std::sin(th);
            F(1,2) =  v * dt * std::cos(th);

            Gu(0,0) = dt * std::cos(th);
            Gu(1,0) = dt * std::sin(th);
            Gu(2,1) = dt;
        } else {
            F(0,2) = (v / w) * (std::cos(th + w*dt) - std::cos(th));
            F(1,2) = (v / w) * (std::sin(th + w*dt) - std::sin(th));

            Gu(0,0) = (1.0 / w) * (std::sin(th + w*dt) - std::sin(th));
            Gu(1,0) = -(1.0 / w) * (std::cos(th + w*dt) - std::cos(th));
            Gu(2,1) = dt;

            Gu(0,1) = (v / (w*w)) * (std::sin(th) - std::sin(th + w*dt)) + (v / w) * (dt * std::cos(th + w*dt));
            Gu(1,1) = (v / (w*w)) * (std::cos(th + w*dt) - std::cos(th)) + (v / w) * (dt * std::sin(th + w*dt));
        }

        P_ = F * P_ * F.transpose() + Gu * Q * Gu.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    void Update(const Measurement& m, const Eigen::Matrix2d& R) {
        if (id_to_slot_.find(m.id) == id_to_slot_.end()) {
            InitializeLandmark(m, R);
        }

        Eigen::Vector2d z(m.range, WrapAngle(m.bearing));
        Eigen::Vector2d zhat;
        Eigen::MatrixXd H;
        ExpectedAndJacobian(m.id, zhat, H);

        Eigen::Vector2d innov = z - zhat;
        innov(1) = WrapAngle(innov(1));

        Eigen::Matrix2d S = (H * P_ * H.transpose() + R);
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        mu_ = mu_ + K * innov;
        mu_(2) = WrapAngle(mu_(2));

        const int D = Dim();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(D, D);
        // Joseph form
        P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    const Eigen::VectorXd& Mean() const { return mu_; }
    const Eigen::MatrixXd& Cov() const { return P_; }

private:
    Eigen::VectorXd mu_;
    Eigen::MatrixXd P_;
    std::unordered_map<int,int> id_to_slot_; // id -> 0..N-1

    int Slot(int id) const { return id_to_slot_.at(id); }

    Eigen::Vector2d Landmark(int id) const {
        int s = Slot(id);
        int idx = 3 + 2*s;
        return Eigen::Vector2d(mu_(idx), mu_(idx+1));
    }

    void ExpectedAndJacobian(int id, Eigen::Vector2d& zhat, Eigen::MatrixXd& H) const {
        const int D = Dim();
        H = Eigen::MatrixXd::Zero(2, D);

        double x = mu_(0), y = mu_(1), th = mu_(2);
        Eigen::Vector2d lm = Landmark(id);
        double dx = lm(0) - x;
        double dy = lm(1) - y;
        double q = dx*dx + dy*dy;
        double r = std::sqrt(q);

        zhat(0) = r;
        zhat(1) = WrapAngle(std::atan2(dy, dx) - th);

        // range
        H(0,0) = -dx / r; H(0,1) = -dy / r;
        // bearing
        H(1,0) =  dy / q; H(1,1) = -dx / q; H(1,2) = -1.0;

        int s = Slot(id);
        int idx = 3 + 2*s;
        // landmark derivatives
        H(0, idx)   =  dx / r; H(0, idx+1) =  dy / r;
        H(1, idx)   = -dy / q; H(1, idx+1) =  dx / q;
    }

    void InitializeLandmark(const Measurement& m, const Eigen::Matrix2d& R) {
        // Inverse observation model:
        //  mx = x + r cos(th + b)
        //  my = y + r sin(th + b)
        double x = mu_(0), y = mu_(1), th = mu_(2);
        double r = m.range, b = m.bearing;
        double ang = th + b;

        Eigen::Vector2d lm;
        lm(0) = x + r * std::cos(ang);
        lm(1) = y + r * std::sin(ang);

        // Jacobians for covariance propagation
        Eigen::Matrix<double,2,3> Gx;
        Gx << 1.0, 0.0, -r * std::sin(ang),
              0.0, 1.0,  r * std::cos(ang);

        Eigen::Matrix2d Gz;
        Gz << std::cos(ang), -r * std::sin(ang),
              std::sin(ang),  r * std::cos(ang);

        // Partition existing covariance
        Eigen::Matrix3d Prr = P_.block<3,3>(0,0);
        Eigen::MatrixXd Pxr = P_.block(0,0, Dim(), 3);

        Eigen::Matrix2d Pmm = Gx * Prr * Gx.transpose() + Gz * R * Gz.transpose();
        Eigen::MatrixXd Pxm = Pxr * Gx.transpose(); // (D x 2)

        // Augment mean
        int D = Dim();
        Eigen::VectorXd mu_new(D + 2);
        mu_new.head(D) = mu_;
        mu_new.segment<2>(D) = lm;
        mu_ = mu_new;

        // Augment covariance
        Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(D + 2, D + 2);
        P_new.block(0,0,D,D) = P_;
        P_new.block(0,D,D,2) = Pxm;
        P_new.block(D,0,2,D) = Pxm.transpose();
        P_new.block(D,D,2,2) = Pmm;
        P_ = P_new;

        id_to_slot_[m.id] = NumLandmarks(); // new last slot
    }
};

int main() {
    EKFSLAM slam;

    Eigen::Matrix2d Q; Q << 0.05*0.05, 0, 0, (2.0 * M_PI/180.0)*(2.0 * M_PI/180.0);
    Eigen::Matrix2d R; R << 0.15*0.15, 0, 0, (3.0 * M_PI/180.0)*(3.0 * M_PI/180.0);

    // Simple synthetic sequence: predict then update to a single landmark id=0
    for (int k = 0; k < 50; ++k) {
        slam.Predict(0.7, 0.1, 0.1, Q);

        // Fake measurement (for demo only; real systems compute from sensors)
        Measurement m;
        m.id = 0;
        m.range = 5.0;
        m.bearing = 0.3;
        slam.Update(m, R);
    }

    std::cout << "State dim: " << slam.Dim() << "\n";
    std::cout << "Mean:\n" << slam.Mean().transpose() << "\n";
    std::cout << "Cov(0:3,0:3):\n" << slam.Cov().block(0,0,3,3) << "\n";
    return 0;
}
