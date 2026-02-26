// Chapter10_Lesson4.cpp
// Robust scan matching (2D) under dynamic obstacles using IRLS-Huber + trimming
// Dependencies: Eigen (header-only). Compile example (Linux/macOS):
//   g++ -O2 -std=c++17 Chapter10_Lesson4.cpp -I /usr/include/eigen3 -o robust_icp
//
// Educational reference implementation (O(NM) nearest neighbors).

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

static Mat2 rot2(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    Mat2 R;
    R << c, -s,
         s,  c;
    return R;
}

static void nearestNeighborsBruteForce(
    const std::vector<Vec2>& A,
    const std::vector<Vec2>& B,
    std::vector<int>& idx,
    std::vector<double>& d2
) {
    idx.resize(A.size());
    d2.resize(A.size());
    for (size_t i = 0; i < A.size(); ++i) {
        double best = std::numeric_limits<double>::infinity();
        int bestj = -1;
        for (size_t j = 0; j < B.size(); ++j) {
            double dist2 = (A[i] - B[j]).squaredNorm();
            if (dist2 < best) { best = dist2; bestj = (int)j; }
        }
        idx[i] = bestj;
        d2[i] = best;
    }
}

static void weightedKabsch2D(
    const std::vector<Vec2>& P,
    const std::vector<Vec2>& Q,
    const std::vector<double>& w,
    Mat2& R, Vec2& t
) {
    double wsum = 1e-12;
    Vec2 pbar(0,0), qbar(0,0);
    for (size_t i = 0; i < P.size(); ++i) {
        wsum += w[i];
        pbar += w[i] * P[i];
        qbar += w[i] * Q[i];
    }
    pbar /= wsum;
    qbar /= wsum;

    Mat2 S = Mat2::Zero();
    for (size_t i = 0; i < P.size(); ++i) {
        Vec2 x = P[i] - pbar;
        Vec2 y = Q[i] - qbar;
        S += w[i] * (x * y.transpose());
    }

    // 2D Kabsch angle
    double num = S(0,1) - S(1,0);
    double den = S(0,0) + S(1,1);
    double theta = std::atan2(num, den);

    R = rot2(theta);
    t = qbar - R * pbar;
}

static double huberWeight(double r, double delta) {
    return (r <= delta) ? 1.0 : (delta / (r + 1e-12));
}

static void robustICP2D(
    const std::vector<Vec2>& src,
    const std::vector<Vec2>& tgt,
    int iters,
    double delta,
    double keepRatio,
    Mat2& R, Vec2& t
) {
    R = Mat2::Identity();
    t = Vec2::Zero();

    std::vector<Vec2> srcW(src.size());
    std::vector<int> nnIdx;
    std::vector<double> d2;

    for (int k = 0; k < iters; ++k) {
        for (size_t i = 0; i < src.size(); ++i) srcW[i] = R * src[i] + t;

        nearestNeighborsBruteForce(srcW, tgt, nnIdx, d2);

        std::vector<double> r(src.size());
        for (size_t i = 0; i < src.size(); ++i) r[i] = std::sqrt(d2[i]);

        // trimming threshold by quantile (approx)
        std::vector<double> rSorted = r;
        size_t qIdx = (size_t)std::floor(keepRatio * (rSorted.size() - 1));
        std::nth_element(rSorted.begin(), rSorted.begin() + qIdx, rSorted.end());
        double thr = rSorted[qIdx];

        std::vector<Vec2> P_in, Q_in;
        std::vector<double> w_in;
        P_in.reserve(src.size());
        Q_in.reserve(src.size());
        w_in.reserve(src.size());

        for (size_t i = 0; i < src.size(); ++i) {
            if (r[i] <= thr) {
                P_in.push_back(src[i]);
                Q_in.push_back(tgt[nnIdx[i]]);
                w_in.push_back(huberWeight(r[i], delta));
            }
        }

        Mat2 dR; Vec2 dt;
        weightedKabsch2D(P_in, Q_in, w_in, dR, dt);

        R = dR * R;
        t = dR * t + dt;
    }
}

static void makeSyntheticScene(
    std::vector<Vec2>& scan1,
    std::vector<Vec2>& scan2,
    Mat2& Rtrue,
    Vec2& ttrue,
    int nStatic=250,
    int nDyn=60,
    int seed=7
) {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    std::normal_distribution<double> gauss(0.0, 1.0);

    auto randu = [&](){ return unif(gen); };
    auto randn = [&](){ return gauss(gen); };

    std::vector<Vec2> staticPts;
    staticPts.reserve(nStatic);

    for (int i = 0; i < nStatic/2; ++i) {
        double ang = 2.0 * M_PI * randu();
        staticPts.push_back(Vec2(2.0*std::cos(ang), 2.0*std::sin(ang)));
    }
    for (int i = 0; i < nStatic/4; ++i) staticPts.push_back(Vec2(-3.0 + 6.0*randu(), -1.5));
    for (int i = 0; i < nStatic/4; ++i) staticPts.push_back(Vec2(1.5, -2.0 + 4.0*randu()));

    Vec2 dyn1c(-0.5, 0.8);
    std::vector<Vec2> dyn1; dyn1.reserve(nDyn);
    for (int i = 0; i < nDyn; ++i) dyn1.push_back(dyn1c + 0.15*Vec2(randn(), randn()));

    double theta = 12.0 * M_PI / 180.0;
    ttrue = Vec2(0.35, -0.10);
    Rtrue = rot2(theta);

    scan1.clear();
    scan1.insert(scan1.end(), staticPts.begin(), staticPts.end());
    scan1.insert(scan1.end(), dyn1.begin(), dyn1.end());
    for (auto& p : scan1) p += 0.02*Vec2(randn(), randn());

    Vec2 dyn2c = dyn1c + Vec2(0.55, -0.25);
    std::vector<Vec2> dyn2; dyn2.reserve(nDyn);
    for (int i = 0; i < nDyn; ++i) dyn2.push_back(dyn2c + 0.15*Vec2(randn(), randn()));

    scan2.clear();
    for (auto& p : staticPts) scan2.push_back(Rtrue * p + ttrue);
    scan2.insert(scan2.end(), dyn2.begin(), dyn2.end());
    for (auto& p : scan2) p += 0.02*Vec2(randn(), randn());
}

static double angleErrorDeg(const Mat2& Rest, const Mat2& Rtrue) {
    Mat2 dR = Rest * Rtrue.transpose();
    double ang = std::atan2(dR(1,0), dR(0,0));
    return std::abs(ang) * 180.0 / M_PI;
}

int main() {
    std::vector<Vec2> scan1, scan2;
    Mat2 Rtrue; Vec2 ttrue;
    makeSyntheticScene(scan1, scan2, Rtrue, ttrue);

    Mat2 R; Vec2 t;
    robustICP2D(scan1, scan2, 25, 0.18, 0.85, R, t);

    std::cout << "True t: " << ttrue.transpose() << "\\n";
    std::cout << "Est  t: " << t.transpose() << "\\n";
    std::cout << "Angle error (deg): " << angleErrorDeg(R, Rtrue) << "\\n";
    std::cout << "Translation error : " << (t - ttrue).norm() << "\\n";
    return 0;
}
