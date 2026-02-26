// Chapter1_Lesson4.cpp
// Sensing–Estimation–Navigation pipeline demo (2D unicycle) using prior + WLS correction.
// Dependencies: C++17. Optional: Eigen (header-only) for linear algebra.
// Build example (with Eigen in include path):
//   g++ -O2 -std=c++17 Chapter1_Lesson4.cpp -I /usr/include/eigen3 -o amr_lesson4
//
// Output: writes a CSV file "traj_ch1_l4.csv" with true and estimated pose per step.
// Typical robotics ecosystem pointers: ROS 2 (rclcpp), Eigen, GTSAM (not required for this demo).

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

static double wrapAngle(double a) {
    const double TWO_PI = 2.0 * M_PI;
    a = std::fmod(a + M_PI, TWO_PI);
    if (a < 0.0) a += TWO_PI;
    return a - M_PI;
}

static Eigen::Vector3d unicycleStep(const Eigen::Vector3d& x, double v, double w, double dt) {
    Eigen::Vector3d xn = x;
    double th = x(2);
    xn(0) = x(0) + v * dt * std::cos(th);
    xn(1) = x(1) + v * dt * std::sin(th);
    xn(2) = wrapAngle(x(2) + w * dt);
    return xn;
}

static Eigen::Vector2d measModel(const Eigen::Vector3d& x, const Eigen::Vector2d& lm) {
    double dx = lm(0) - x(0);
    double dy = lm(1) - x(1);
    double r  = std::hypot(dx, dy);
    double b  = wrapAngle(std::atan2(dy, dx) - x(2));
    return Eigen::Vector2d(r, b);
}

static Eigen::Matrix<double,2,3> measJacobian(const Eigen::Vector3d& x, const Eigen::Vector2d& lm) {
    double dx = lm(0) - x(0);
    double dy = lm(1) - x(1);
    double q = dx*dx + dy*dy;
    double r = std::sqrt(std::max(q, 1e-12));

    q = std::max(q, 1e-12);

    Eigen::Matrix<double,2,3> H;
    // range derivatives
    H(0,0) = -dx / r;
    H(0,1) = -dy / r;
    H(0,2) = 0.0;
    // bearing derivatives
    H(1,0) =  dy / q;
    H(1,1) = -dx / q;
    H(1,2) = -1.0;
    return H;
}

static void wlsUpdateIterated(
    const Eigen::Vector3d& x_prior, const Eigen::Matrix3d& P_prior,
    const std::vector<Eigen::Vector2d>& z_list, const std::vector<Eigen::Vector2d>& lm_list,
    const Eigen::Matrix2d& R, int iters,
    Eigen::Vector3d& x_post, Eigen::Matrix3d& P_post
) {
    if (z_list.empty()) {
        x_post = x_prior;
        P_post = P_prior;
        return;
    }

    int m = static_cast<int>(z_list.size());
    Eigen::VectorXd z(2*m);
    for (int i = 0; i < m; ++i) {
        z.segment<2>(2*i) = z_list[i];
    }

    Eigen::MatrixXd Rbig = Eigen::MatrixXd::Zero(2*m, 2*m);
    for (int i = 0; i < m; ++i) {
        Rbig.block<2,2>(2*i, 2*i) = R;
    }
    Eigen::MatrixXd Rinv = Rbig.inverse();
    Eigen::Matrix3d Pinv = P_prior.inverse();

    Eigen::Vector3d x = x_prior;

    for (int it = 0; it < iters; ++it) {
        Eigen::VectorXd h(2*m);
        Eigen::MatrixXd H(2*m, 3);

        for (int i = 0; i < m; ++i) {
            Eigen::Vector2d hi = measModel(x, lm_list[i]);
            h.segment<2>(2*i) = hi;
            H.block<2,3>(2*i, 0) = measJacobian(x, lm_list[i]);
        }

        Eigen::VectorXd res = z - h;
        for (int i = 0; i < m; ++i) {
            res(2*i + 1) = wrapAngle(res(2*i + 1));
        }

        Eigen::Matrix3d A = Pinv + H.transpose() * Rinv * H;
        Eigen::Vector3d b = H.transpose() * Rinv * res + Pinv * (x_prior - x);

        Eigen::Vector3d delta = A.ldlt().solve(b);
        x = x + delta;
        x(2) = wrapAngle(x(2));
    }

    // Final covariance
    Eigen::MatrixXd H(2*m, 3);
    for (int i = 0; i < m; ++i) {
        H.block<2,3>(2*i, 0) = measJacobian(x, lm_list[i]);
    }
    Eigen::Matrix3d Pinv_post = Pinv + H.transpose() * Rinv * H;
    P_post = Pinv_post.inverse();
    x_post = x;
}

int main() {
    std::mt19937 rng(7);
    std::normal_distribution<double> n01(0.0, 1.0);

    // Landmarks (known map)
    std::vector<Eigen::Vector2d> landmarks;
    landmarks.emplace_back(5.0, 0.0);
    landmarks.emplace_back(6.0, 6.0);
    landmarks.emplace_back(0.0, 6.0);

    const double dt = 0.1;
    const int N = 350;

    Eigen::Vector3d x_true(0.0, 0.0, 0.0);
    Eigen::Vector3d x_hat(0.0, 0.0, 0.0);

    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    P(0,0) = 0.15*0.15;
    P(1,1) = 0.15*0.15;
    P(2,2) = std::pow(8.0*M_PI/180.0, 2);

    const double sigma_v = 0.08;
    const double sigma_w = 3.0*M_PI/180.0;
    const double sigma_r = 0.12;
    const double sigma_b = 2.0*M_PI/180.0;

    Eigen::Vector2d goal(7.0, 7.0);
    const double k_heading = 1.6;
    const double v_max = 0.9;

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = sigma_r*sigma_r;
    R(1,1) = sigma_b*sigma_b;

    std::ofstream csv("traj_ch1_l4.csv");
    csv << "k,x_true,y_true,th_true,x_hat,y_hat,th_hat\n";

    for (int k = 0; k < N; ++k) {
        // Navigation uses estimate
        double dx = goal(0) - x_hat(0);
        double dy = goal(1) - x_hat(1);
        double dist = std::hypot(dx, dy);
        double desired = std::atan2(dy, dx);
        double heading_err = wrapAngle(desired - x_hat(2));

        double v_cmd = v_max * std::tanh(dist);
        double w_cmd = k_heading * heading_err;

        // True motion includes mild slip
        double slip = (0.35*M_PI/180.0) * n01(rng);
        x_true = unicycleStep(x_true, v_cmd, w_cmd + slip/dt, dt);

        // Odometry
        double v_meas = v_cmd + sigma_v * n01(rng);
        double w_meas = w_cmd + sigma_w * n01(rng);

        Eigen::Vector3d x_prior = unicycleStep(x_hat, v_meas, w_meas, dt);

        double th = x_hat(2);
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0,2) = -v_meas*dt*std::sin(th);
        F(1,2) =  v_meas*dt*std::cos(th);

        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
        Q(0,0) = std::pow(sigma_v*dt, 2);
        Q(1,1) = std::pow(sigma_v*dt, 2);
        Q(2,2) = std::pow(sigma_w*dt, 2);

        Eigen::Matrix3d P_prior = F * P * F.transpose() + Q;

        // Landmark measurements
        std::vector<Eigen::Vector2d> z_list;
        std::vector<Eigen::Vector2d> lm_list;
        const double max_range = 8.0;

        for (const auto& lm : landmarks) {
            Eigen::Vector2d z_true = measModel(x_true, lm);
            if (z_true(0) <= max_range) {
                Eigen::Vector2d z_noisy = z_true;
                z_noisy(0) += sigma_r * n01(rng);
                z_noisy(1)  = wrapAngle(z_noisy(1) + sigma_b * n01(rng));
                z_list.push_back(z_noisy);
                lm_list.push_back(lm);
            }
        }

        // WLS correction
        Eigen::Vector3d x_post;
        Eigen::Matrix3d P_post;
        wlsUpdateIterated(x_prior, P_prior, z_list, lm_list, R, 2, x_post, P_post);

        x_hat = x_post;
        P = P_post;

        csv << k << "," << x_true(0) << "," << x_true(1) << "," << x_true(2) << ","
            << x_hat(0)  << "," << x_hat(1)  << "," << x_hat(2)  << "\n";
    }

    std::cout << "Wrote traj_ch1_l4.csv\n";
    return 0;
}
