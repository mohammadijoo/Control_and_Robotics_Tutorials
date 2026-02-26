// Chapter19_Lesson1.cpp
// Metrics for Localization and SLAM (Autonomous Mobile Robots)
// C++ reference implementation using Eigen (SE(2) alignment, ATE, RPE)

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <numeric>
#include <Eigen/Dense>

struct Pose2 {
    double x;
    double y;
    double yaw;
};

double wrapToPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

struct SE2AlignResult {
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    double theta;
};

SE2AlignResult alignSE2(const std::vector<Pose2>& gt, const std::vector<Pose2>& est) {
    const int N = static_cast<int>(gt.size());
    Eigen::Vector2d c_gt(0, 0), c_est(0, 0);

    for (int i = 0; i < N; ++i) {
        c_gt += Eigen::Vector2d(gt[i].x, gt[i].y);
        c_est += Eigen::Vector2d(est[i].x, est[i].y);
    }
    c_gt /= N;
    c_est /= N;

    double s = 0.0, c = 0.0;
    for (int i = 0; i < N; ++i) {
        Eigen::Vector2d X(est[i].x, est[i].y); X -= c_est;
        Eigen::Vector2d Y(gt[i].x, gt[i].y);   Y -= c_gt;
        s += X.x() * Y.y() - X.y() * Y.x();
        c += X.x() * Y.x() + X.y() * Y.y();
    }
    double theta = std::atan2(s, c);

    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    Eigen::Vector2d t = c_gt - R * c_est;
    return {R, t, theta};
}

std::vector<Pose2> applySE2(const std::vector<Pose2>& est, const SE2AlignResult& A) {
    std::vector<Pose2> out = est;
    for (size_t i = 0; i < est.size(); ++i) {
        Eigen::Vector2d p(est[i].x, est[i].y);
        Eigen::Vector2d q = A.R * p + A.t;
        out[i].x = q.x();
        out[i].y = q.y();
        out[i].yaw = wrapToPi(est[i].yaw + A.theta);
    }
    return out;
}

double ateRMSE(const std::vector<Pose2>& gt, const std::vector<Pose2>& est) {
    double s = 0.0;
    for (size_t i = 0; i < gt.size(); ++i) {
        double dx = gt[i].x - est[i].x;
        double dy = gt[i].y - est[i].y;
        s += dx * dx + dy * dy;
    }
    return std::sqrt(s / gt.size());
}

std::pair<double,double> rpeRMSE(const std::vector<Pose2>& gt, const std::vector<Pose2>& est, int delta) {
    std::vector<double> et, er;
    for (int k = 0; k + delta < static_cast<int>(gt.size()); ++k) {
        double dgx = gt[k + delta].x - gt[k].x;
        double dgy = gt[k + delta].y - gt[k].y;
        double dex = est[k + delta].x - est[k].x;
        double dey = est[k + delta].y - est[k].y;

        double dt = std::hypot(dgx - dex, dgy - dey);
        et.push_back(dt);

        double dpg = wrapToPi(gt[k + delta].yaw - gt[k].yaw);
        double dpe = wrapToPi(est[k + delta].yaw - est[k].yaw);
        er.push_back(std::fabs(wrapToPi(dpg - dpe)));
    }

    auto rmse = [](const std::vector<double>& v) {
        double s = 0.0;
        for (double x : v) s += x * x;
        return std::sqrt(s / v.size());
    };
    return {rmse(et), rmse(er)};
}

int main() {
    std::mt19937 gen(19);
    std::normal_distribution<double> nxy(0.0, 0.03);
    std::normal_distribution<double> nyaw(0.0, 0.01);

    const int N = 200;
    std::vector<Pose2> gt(N), est(N);

    double theta0 = 7.0 * M_PI / 180.0;
    Eigen::Matrix2d R0;
    R0 << std::cos(theta0), -std::sin(theta0),
          std::sin(theta0),  std::cos(theta0);
    Eigen::Vector2d t0(1.5, -0.8);

    std::vector<double> tx(N), ty(N);
    for (int i = 0; i < N; ++i) {
        double t = 20.0 * i / (N - 1.0);
        tx[i] = 0.5 * t;
        ty[i] = 2.0 * std::sin(0.4 * t);
    }
    for (int i = 0; i < N; ++i) {
        double dx = (i == 0 ? tx[1] - tx[0] : tx[i] - tx[i - 1]);
        double dy = (i == 0 ? ty[1] - ty[0] : ty[i] - ty[i - 1]);
        gt[i].x = tx[i];
        gt[i].y = ty[i];
        gt[i].yaw = std::atan2(dy, dx);

        Eigen::Vector2d p_gt(gt[i].x, gt[i].y);
        Eigen::Vector2d drift(0.01 * (20.0 * i / (N - 1.0)), -0.004 * (20.0 * i / (N - 1.0)));
        Eigen::Vector2d p_est = R0.transpose() * (p_gt - t0) + drift + Eigen::Vector2d(nxy(gen), nxy(gen));

        est[i].x = p_est.x();
        est[i].y = p_est.y();
        est[i].yaw = wrapToPi(gt[i].yaw - theta0 + 0.01 * std::sin(0.2 * (20.0 * i / (N - 1.0))) + nyaw(gen));
    }

    SE2AlignResult A = alignSE2(gt, est);
    auto estAligned = applySE2(est, A);

    double ate = ateRMSE(gt, estAligned);
    auto rpe = rpeRMSE(gt, estAligned, 5);

    std::cout << "=== Chapter19 Lesson1 Metrics Demo (C++) ===\n";
    std::cout << "ATE RMSE [m]: " << ate << "\n";
    std::cout << "RPE translational RMSE [m] (delta=5): " << rpe.first << "\n";
    std::cout << "RPE rotational RMSE [rad] (delta=5): " << rpe.second << "\n";
    std::cout << "Estimated global alignment theta [deg]: " << (A.theta * 180.0 / M_PI) << "\n";
    return 0;
}
