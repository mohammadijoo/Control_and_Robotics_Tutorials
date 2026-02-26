// Chapter20_Lesson3.cpp
// Localization + Mapping Integration (Capstone AMR)
// Educational Eigen-based example: EKF prediction/correction + occupancy log-odds update.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include <algorithm>

struct Config {
    double dt = 0.1;
    double qv = 0.02;
    double qw = 0.03;
    double rr = 0.15;
    double rb = 0.03;
    double gridRes = 0.20;
    int gridSize = 120;
    double lOcc = 0.85;
    double lFree = -0.40;
    double lMin = -4.0;
    double lMax = 4.0;
};

static double WrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class EKFLocalizer {
public:
    explicit EKFLocalizer(const Config& cfg) : cfg_(cfg) {
        x_ << 1.0, 1.0, 0.0;
        P_.setZero();
        P_(0,0) = 0.05; P_(1,1) = 0.05; P_(2,2) = 0.02;
    }

    void Predict(double v, double w) {
        const double dt = cfg_.dt;
        const double x = x_(0), y = x_(1), th = x_(2);

        x_(0) = x + v * std::cos(th) * dt;
        x_(1) = y + v * std::sin(th) * dt;
        x_(2) = WrapAngle(th + w * dt);

        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0,2) = -v * std::sin(th) * dt;
        F(1,2) =  v * std::cos(th) * dt;

        Eigen::Matrix<double,3,2> G;
        G.setZero();
        G(0,0) = std::cos(th) * dt;
        G(1,0) = std::sin(th) * dt;
        G(2,1) = dt;

        Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
        Q(0,0) = cfg_.qv * cfg_.qv;
        Q(1,1) = cfg_.qw * cfg_.qw;

        P_ = F * P_ * F.transpose() + G * Q * G.transpose();
    }

    void CorrectLandmark(double zRange, double zBearing, const Eigen::Vector2d& lm) {
        const double dx = lm(0) - x_(0);
        const double dy = lm(1) - x_(1);
        const double q = dx*dx + dy*dy;
        if (q < 1e-12) return;

        Eigen::Vector2d zhat;
        zhat(0) = std::sqrt(q);
        zhat(1) = WrapAngle(std::atan2(dy, dx) - x_(2));

        Eigen::Matrix<double,2,3> H;
        H << -dx/std::sqrt(q), -dy/std::sqrt(q), 0.0,
              dy/q,            -dx/q,           -1.0;

        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R(0,0) = cfg_.rr * cfg_.rr;
        R(1,1) = cfg_.rb * cfg_.rb;

        Eigen::Matrix2d S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double,3,2> K = P_ * H.transpose() * S.inverse();

        Eigen::Vector2d innov;
        innov(0) = zRange - zhat(0);
        innov(1) = WrapAngle(zBearing - zhat(1));

        double d2 = innov.transpose() * S.inverse() * innov;
        if (d2 > 9.21) return;

        x_ = x_ + K * innov;
        x_(2) = WrapAngle(x_(2));

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P_ = (I - K*H) * P_ * (I - K*H).transpose() + K * R * K.transpose();
    }

    const Eigen::Vector3d& state() const { return x_; }
    const Eigen::Matrix3d& cov() const { return P_; }

private:
    Config cfg_;
    Eigen::Vector3d x_;
    Eigen::Matrix3d P_;
};

class OccupancyGrid {
public:
    explicit OccupancyGrid(const Config& cfg) : cfg_(cfg) {
        logOdds_.assign(cfg_.gridSize * cfg_.gridSize, 0.0);
    }

    std::pair<int,int> WorldToGrid(double x, double y) const {
        return {static_cast<int>(x / cfg_.gridRes), static_cast<int>(y / cfg_.gridRes)};
    }

    bool InBounds(int gx, int gy) const {
        return gx >= 0 && gy >= 0 && gx < cfg_.gridSize && gy < cfg_.gridSize;
    }

    std::vector<std::pair<int,int>> Bresenham(int x0, int y0, int x1, int y1) const {
        std::vector<std::pair<int,int>> pts;
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        int x = x0, y = y0;
        while (true) {
            pts.push_back({x,y});
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
        return pts;
    }

    void UpdateRay(double rx, double ry, double ex, double ey, bool hit) {
        auto [g0x, g0y] = WorldToGrid(rx, ry);
        auto [g1x, g1y] = WorldToGrid(ex, ey);
        if (!InBounds(g0x, g0y) || !InBounds(g1x, g1y)) return;

        auto ray = Bresenham(g0x, g0y, g1x, g1y);
        for (size_t i = 0; i + 1 < ray.size(); ++i) {
            Accumulate(ray[i].first, ray[i].second, cfg_.lFree);
        }
        if (hit && !ray.empty()) {
            Accumulate(ray.back().first, ray.back().second, cfg_.lOcc);
        }
    }

    void PrintSummary() const {
        double mn = 1e9, mx = -1e9, avg = 0.0;
        for (double l : logOdds_) {
            double p = 1.0 / (1.0 + std::exp(-l));
            mn = std::min(mn, p);
            mx = std::max(mx, p);
            avg += p;
        }
        avg /= static_cast<double>(logOdds_.size());
        std::cout << "Map occupancy prob stats: min=" << mn
                  << " max=" << mx << " mean=" << avg << "\n";
    }

private:
    void Accumulate(int gx, int gy, double delta) {
        if (!InBounds(gx, gy)) return;
        double& cell = logOdds_[gy * cfg_.gridSize + gx];
        cell = std::clamp(cell + delta, cfg_.lMin, cfg_.lMax);
    }

    Config cfg_;
    std::vector<double> logOdds_;
};

static std::mt19937_64 rng(7);
static std::normal_distribution<double> n01(0.0, 1.0);

int main() {
    Config cfg;
    EKFLocalizer ekf(cfg);
    OccupancyGrid grid(cfg);

    Eigen::Vector3d xTrue(1.0, 1.0, 0.0);
    std::vector<Eigen::Vector2d> landmarks = {
        {2.0, 10.0}, {10.0, 2.5}, {9.5, 10.5}, {2.0, 5.5}
    };

    auto randn = [](double s){ return s * n01(rng); };

    double sqErrPos = 0.0, sqErrYaw = 0.0;
    int n = 0;

    for (int k = 0; k < 240; ++k) {
        double vCmd = 0.35 + 0.05 * std::sin(0.05 * k);
        double wCmd = 0.25 * std::sin(0.03 * k);

        // Truth
        xTrue(0) += vCmd * std::cos(xTrue(2)) * cfg.dt;
        xTrue(1) += vCmd * std::sin(xTrue(2)) * cfg.dt;
        xTrue(2) = WrapAngle(xTrue(2) + wCmd * cfg.dt + randn(0.005));

        // EKF prediction
        ekf.Predict(vCmd + randn(0.02), wCmd + randn(0.01));

        // Landmark corrections
        if (k % 5 == 0) {
            for (const auto& lm : landmarks) {
                double dx = lm(0) - xTrue(0);
                double dy = lm(1) - xTrue(1);
                double range = std::sqrt(dx*dx + dy*dy);
                if (range < 7.5) {
                    double zRange = range + randn(0.08);
                    double zBearing = WrapAngle(std::atan2(dy, dx) - xTrue(2) + randn(0.02));
                    ekf.CorrectLandmark(zRange, zBearing, lm);
                }
            }
        }

        // Lightweight synthetic rays (front, left, right) for grid update
        const auto est = ekf.state();
        std::vector<double> beamAngles = {-0.7, 0.0, 0.7};
        for (double a : beamAngles) {
            double rr = 5.0; // pretend obstacle at fixed distance in demo
            double ex = est(0) + rr * std::cos(est(2) + a);
            double ey = est(1) + rr * std::sin(est(2) + a);
            grid.UpdateRay(est(0), est(1), ex, ey, true);
        }

        const auto xe = ekf.state();
        double ex = xe(0) - xTrue(0);
        double ey = xe(1) - xTrue(1);
        sqErrPos += ex*ex + ey*ey;
        sqErrYaw += std::pow(WrapAngle(xe(2) - xTrue(2)), 2);
        ++n;
    }

    std::cout << "Final estimated pose: " << ekf.state().transpose() << "\n";
    std::cout << "Final covariance diagonal: " << ekf.cov().diagonal().transpose() << "\n";
    std::cout << "Position RMSE [m]: " << std::sqrt(sqErrPos / n) << "\n";
    std::cout << "Yaw RMSE [rad]: " << std::sqrt(sqErrYaw / n) << "\n";
    grid.PrintSummary();

    return 0;
}
