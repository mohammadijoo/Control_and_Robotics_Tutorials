// Chapter10_Lesson5.cpp
/*
Autonomous Mobile Robots — Chapter 10, Lesson 5
Lab: ICP-Based Motion Estimation (2D)

A compact from-scratch 2D point-to-point ICP using:
- brute-force nearest neighbors (simple but O(N^2))
- closed-form 2D rotation estimate from cross-covariance
- optional distance gating and trimming

Dependencies: Eigen (header-only linear algebra)
Compile (example):
  g++ -O2 -std=c++17 Chapter10_Lesson5.cpp -I /path/to/eigen -o icp2d
Run:
  ./icp2d
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

static Mat2 rot2(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    Mat2 R;
    R << c, -s,
         s,  c;
    return R;
}

static std::vector<Vec2> applyTransform(const std::vector<Vec2>& P, const Mat2& R, const Vec2& t) {
    std::vector<Vec2> out;
    out.reserve(P.size());
    for (const auto& p : P) out.push_back(R * p + t);
    return out;
}

static double norm2(const Vec2& v) { return v.squaredNorm(); }

static Vec2 centroid(const std::vector<Vec2>& P, const std::vector<double>* w = nullptr) {
    double ws = 0.0;
    Vec2 mu(0.0, 0.0);
    if (!w) {
        for (const auto& p : P) mu += p;
        return mu / std::max<size_t>(1, P.size());
    }
    for (size_t i = 0; i < P.size(); ++i) {
        mu += (*w)[i] * P[i];
        ws += (*w)[i];
    }
    return mu / (ws + 1e-12);
}

// Closed-form 2D optimal rotation from correspondences (no SVD needed).
// Given centered pairs (x_i, y_i), define:
//   a = sum( x_i dot y_i )
//   b = sum( x_i cross y_i )  (2D scalar cross: x_x y_y - x_y y_x)
// theta = atan2(b, a), R(theta) is optimal for least squares.
static void bestFit2D(const std::vector<Vec2>& src,
                      const std::vector<Vec2>& dst,
                      Mat2& R, Vec2& t,
                      const std::vector<double>* w = nullptr) {
    Vec2 mu_s = centroid(src, w);
    Vec2 mu_d = centroid(dst, w);

    double a = 0.0, b = 0.0;
    double ws = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
        Vec2 xs = src[i] - mu_s;
        Vec2 yd = dst[i] - mu_d;
        double wi = (w ? (*w)[i] : 1.0);
        a += wi * (xs.dot(yd));
        b += wi * (xs.x() * yd.y() - xs.y() * yd.x());
        ws += wi;
    }
    (void)ws;
    double theta = std::atan2(b, a);
    R = rot2(theta);
    t = mu_d - R * mu_s;
}

static size_t nearestIndex(const Vec2& p, const std::vector<Vec2>& Q) {
    size_t best = 0;
    double bestd = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < Q.size(); ++j) {
        double d = (Q[j] - p).squaredNorm();
        if (d < bestd) { bestd = d; best = j; }
    }
    return best;
}

struct ICPResult {
    Mat2 R;
    Vec2 t;
    int iters;
    double rmse;
};

static ICPResult icp2D(const std::vector<Vec2>& src0,
                       const std::vector<Vec2>& dst0,
                       Mat2 R, Vec2 t,
                       int maxIter = 60,
                       double tol = 1e-7,
                       double rejectDist = 0.5,   // meters
                       double trimFraction = 0.85 // keep best fraction
) {
    std::vector<double> rmseHist;
    rmseHist.reserve(maxIter);

    double prev = std::numeric_limits<double>::infinity();

    for (int it = 0; it < maxIter; ++it) {
        auto srcT = applyTransform(src0, R, t);

        // Build correspondences
        std::vector<Vec2> srcM, dstM;
        std::vector<double> dists;
        srcM.reserve(srcT.size());
        dstM.reserve(srcT.size());
        dists.reserve(srcT.size());

        for (size_t i = 0; i < srcT.size(); ++i) {
            size_t j = nearestIndex(srcT[i], dst0);
            double d = (dst0[j] - srcT[i]).norm();
            if (d <= rejectDist) {
                srcM.push_back(srcT[i]);
                dstM.push_back(dst0[j]);
                dists.push_back(d);
            }
        }

        if (srcM.size() < 3) throw std::runtime_error("Too few correspondences after gating.");

        // Trimming: keep smallest distances
        std::vector<size_t> idx(srcM.size());
        for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b){ return dists[a] < dists[b]; });
        size_t k = std::max<size_t>(3, static_cast<size_t>(trimFraction * idx.size()));

        std::vector<Vec2> srcK, dstK;
        srcK.reserve(k); dstK.reserve(k);
        for (size_t ii = 0; ii < k; ++ii) {
            srcK.push_back(srcM[idx[ii]]);
            dstK.push_back(dstM[idx[ii]]);
        }

        Mat2 dR; Vec2 dt;
        bestFit2D(srcK, dstK, dR, dt);

        // Compose update
        R = dR * R;
        t = dR * t + dt;

        // RMSE on the kept set
        double sse = 0.0;
        for (size_t i = 0; i < k; ++i) {
            Vec2 e = (dR * srcK[i] + dt) - dstK[i];
            sse += e.squaredNorm();
        }
        double rmse = std::sqrt(sse / std::max<size_t>(1, k));
        rmseHist.push_back(rmse);

        if (std::abs(prev - rmse) < tol) {
            return {R, t, it + 1, rmse};
        }
        prev = rmse;
    }
    return {R, t, maxIter, rmseHist.empty() ? 0.0 : rmseHist.back()};
}

static std::vector<Vec2> simulateWorld(size_t n, int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(0.0, 8.0), uy(0.0, 6.0);

    std::vector<Vec2> P;
    P.reserve(n);

    // L-corner walls
    for (size_t i = 0; i < n/4; ++i) P.push_back(Vec2(ux(rng), 0.0));
    for (size_t i = 0; i < n/4; ++i) P.push_back(Vec2(0.0, uy(rng)));

    // Scatter
    for (size_t i = 0; i < n/2; ++i) P.push_back(Vec2(ux(rng), uy(rng)));
    return P;
}

static std::vector<Vec2> scanFromPose(const std::vector<Vec2>& world,
                                      const Mat2& R_wb, const Vec2& t_wb,
                                      double noiseStd, int seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, noiseStd);

    Mat2 R_bw = R_wb.transpose();
    std::vector<Vec2> scan;
    scan.reserve(world.size());

    double maxRange = 10.0;
    double fov = 270.0 * M_PI / 180.0;

    for (const auto& Xw : world) {
        Vec2 Xb = R_bw * (Xw - t_wb);
        double r = Xb.norm();
        double a = std::atan2(Xb.y(), Xb.x());
        if (r <= maxRange && std::abs(a) <= fov/2.0) {
            Xb.x() += n01(rng);
            Xb.y() += n01(rng);
            scan.push_back(Xb);
        }
    }

    // A few outliers
    std::uniform_real_distribution<double> uo(-2.0, 2.0);
    size_t kout = std::max<size_t>(5, scan.size()/40);
    for (size_t i = 0; i < kout; ++i) scan.push_back(Vec2(uo(rng), uo(rng)));

    return scan;
}

static double angleDeg(const Mat2& R) {
    return std::atan2(R(1,0), R(0,0)) * 180.0 / M_PI;
}

int main() {
    auto world = simulateWorld(700, 7);

    Mat2 R0 = rot2(0.0);
    Vec2 t0(0.0, 0.0);

    double thetaGT = 6.0 * M_PI / 180.0;
    Mat2 R1 = rot2(thetaGT);
    Vec2 t1(0.35, 0.10);

    auto scanK  = scanFromPose(world, R0, t0, 0.01, 1);
    auto scanK1 = scanFromPose(world, R1, t1, 0.01, 2);

    // Initial guess (odometry-like)
    Mat2 Rinit = rot2(3.0 * M_PI / 180.0);
    Vec2 tinit(0.20, 0.0);

    auto res = icp2D(scanK1, scanK, Rinit, tinit, 60, 1e-7, 0.5, 0.85);

    std::cout << "=== ICP-Based Motion Estimation (2D) ===\n";
    std::cout << "Ground truth: theta = " << (thetaGT * 180.0 / M_PI) << " deg, "
              << "t = [" << t1.x() << ", " << t1.y() << "]\n";
    std::cout << "Estimated   : theta = " << angleDeg(res.R) << " deg, "
              << "t = [" << res.t.x() << ", " << res.t.y() << "]\n";
    std::cout << "Iterations  : " << res.iters << ", final RMSE ≈ " << res.rmse << "\n";

    return 0;
}
