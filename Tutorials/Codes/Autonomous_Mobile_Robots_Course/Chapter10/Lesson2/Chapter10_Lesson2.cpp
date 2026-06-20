// Chapter10_Lesson2.cpp
// ICP Variants for Mobile Robots (2D SE(2) focus)
//
// Implements point-to-point ICP (SVD/Procrustes) and point-to-line ICP (linearized LS).
// Notes:
//  - For production, prefer kd-tree (nanoflann) or PCL (pcl::IterativeClosestPoint).
//  - This file is self-contained except for Eigen.
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter10_Lesson2.cpp -I /path/to/eigen -o icp_demo
//
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <limits>
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

struct ICPResult {
    Mat2 R;
    Vec2 t;
    double theta;
    std::vector<double> cost;
};

static void estimateNormals2D(const std::vector<Vec2>& dst, std::vector<Vec2>& normals) {
    int M = (int)dst.size();
    normals.resize(M);
    for (int i=0; i<M; ++i) {
        const Vec2& prev = dst[(i-1+M)%M];
        const Vec2& next = dst[(i+1)%M];
        Vec2 tang = next - prev;
        Vec2 n(-tang.y(), tang.x());
        double nn = n.norm();
        normals[i] = (nn > 1e-12) ? (n/nn) : Vec2(1.0, 0.0);
    }
}

static int nearestBrute(const Vec2& p, const std::vector<Vec2>& dst, double& bestDist) {
    int bestIdx = -1;
    bestDist = std::numeric_limits<double>::infinity();
    for (int j=0; j<(int)dst.size(); ++j) {
        double d = (p - dst[j]).squaredNorm();
        if (d < bestDist) { bestDist = d; bestIdx = j; }
    }
    bestDist = std::sqrt(bestDist);
    return bestIdx;
}

static void solvePointToPoint(const std::vector<Vec2>& P, const std::vector<Vec2>& Q, Mat2& R, Vec2& t) {
    // Unweighted 2D Procrustes
    Vec2 pbar(0,0), qbar(0,0);
    for (size_t i=0; i<P.size(); ++i) { pbar += P[i]; qbar += Q[i]; }
    pbar /= (double)P.size();
    qbar /= (double)Q.size();

    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (size_t i=0; i<P.size(); ++i) {
        Vec2 pp = P[i] - pbar;
        Vec2 qq = Q[i] - qbar;
        H += pp * qq.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    Eigen::Matrix2d Rtmp = V * U.transpose();
    if (Rtmp.determinant() < 0) {
        V.col(1) *= -1.0;
        Rtmp = V * U.transpose();
    }
    R = Rtmp;
    t = qbar - R * pbar;
}

static void solvePointToLine(
    const std::vector<Vec2>& src, const std::vector<Vec2>& dst, const std::vector<Vec2>& n,
    const Mat2& R0, const Vec2& t0,
    Mat2& Rdelta, Vec2& tdelta, double& mean_r2
) {
    // Linearized LS in delta = [dx, dy, dtheta]
    int N = (int)src.size();
    Eigen::MatrixXd A(N, 3);
    Eigen::VectorXd b(N);
    mean_r2 = 0.0;

    for (int i=0; i<N; ++i) {
        Vec2 p = R0 * src[i] + t0;
        Vec2 e = p - dst[i];
        double r = n[i].dot(e);
        // Jacobian
        A(i,0) = n[i].x();
        A(i,1) = n[i].y();
        Vec2 perp(-src[i].y(), src[i].x());
        double jtheta = n[i].dot(R0 * perp);
        A(i,2) = jtheta;
        b(i) = -r;
        mean_r2 += r*r;
    }
    mean_r2 /= (double)N;

    Eigen::Vector3d delta = A.colPivHouseholderQr().solve(b);
    double dx = delta(0), dy = delta(1), dtheta = delta(2);
    Rdelta = rot2(dtheta);
    tdelta = Vec2(dx, dy);
}

static ICPResult icpSE2(
    const std::vector<Vec2>& src_in,
    const std::vector<Vec2>& dst,
    const std::string& variant,
    int maxIter,
    double tol,
    double rejectDist
) {
    Mat2 R = rot2(0.0);
    Vec2 t(0.0, 0.0);

    std::vector<Vec2> normals;
    if (variant == "p2l") estimateNormals2D(dst, normals);

    ICPResult res;
    res.R = R; res.t = t;

    double prevCost = std::numeric_limits<double>::infinity();

    for (int it=0; it<maxIter; ++it) {
        std::vector<Vec2> P, Q, Nn;
        P.reserve(src_in.size());
        Q.reserve(src_in.size());
        Nn.reserve(src_in.size());

        // correspondences
        for (size_t i=0; i<src_in.size(); ++i) {
            Vec2 p = R * src_in[i] + t;
            double d; int j = nearestBrute(p, dst, d);
            if (j < 0) continue;
            if (rejectDist > 0 && d > rejectDist) continue;
            P.push_back(src_in[i]);
            Q.push_back(dst[j]);
            if (variant == "p2l") Nn.push_back(normals[j]);
        }

        if (P.size() < 3) break;

        double cost = 0.0;

        if (variant == "p2p") {
            Mat2 Rnew; Vec2 tnew;
            solvePointToPoint(P, Q, Rnew, tnew);
            R = Rnew; t = tnew;

            for (size_t i=0; i<P.size(); ++i) {
                Vec2 e = R*P[i] + t - Q[i];
                cost += e.squaredNorm();
            }
            cost /= (double)P.size();
        } else {
            Mat2 Rdelta; Vec2 tdelta;
            double mean_r2;
            solvePointToLine(P, Q, Nn, R, t, Rdelta, tdelta, mean_r2);
            // compose delta o current
            R = Rdelta * R;
            t = Rdelta * t + tdelta;
            cost = mean_r2;
        }

        res.cost.push_back(cost);
        if (std::abs(prevCost - cost) < tol) break;
        prevCost = cost;
    }

    res.R = R;
    res.t = t;
    res.theta = std::atan2(R(1,0), R(0,0));
    return res;
}

int main() {
    // Synthetic demo
    std::vector<Vec2> dst;
    int M = 300;
    dst.reserve(M);
    for (int i=0; i<M; ++i) {
        double a = 2.0*M_PI * (double)i/(double)M;
        double x = 2.0*std::cos(a) + 0.3*std::cos(5*a);
        double y = 1.0*std::sin(a) + 0.2*std::sin(3*a);
        dst.emplace_back(x,y);
    }

    double trueTheta = 0.25;
    Vec2 trueT(0.8, -0.4);
    Mat2 Rt = rot2(trueTheta);

    std::vector<Vec2> src;
    src.reserve(M);
    for (int i=0; i<M; ++i) {
        Vec2 p = Rt*dst[i] + trueT;
        src.push_back(p);
    }

    ICPResult r = icpSE2(src, dst, "p2l", 60, 1e-7, 0.5);
    std::cout << "Estimated theta = " << r.theta << "\n";
    std::cout << "Estimated t = [" << r.t.x() << ", " << r.t.y() << "]\n";
    std::cout << "True theta = " << trueTheta << "\n";
    std::cout << "True t = [" << trueT.x() << ", " << trueT.y() << "]\n";
    return 0;
}
