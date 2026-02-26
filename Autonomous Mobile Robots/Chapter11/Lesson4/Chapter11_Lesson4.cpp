// Chapter11_Lesson4.cpp
// Chapter 11 (SLAM I) — Lesson 4: Data Association Challenges
// Minimal C++ demo: chi-square gating + nearest-neighbor association (range-bearing).
//
// Dependency: Eigen (header-only)

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

static double wrapToPi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

// Wilson–Hilferty chi2 inverse (approx) for gating.
static double erfinvApprox(double x) {
    x = std::max(-0.999999, std::min(0.999999, x));
    const double a = 0.147;
    const double ln = std::log(1.0 - x * x);
    const double t  = 2.0 / (M_PI * a) + ln / 2.0;
    const double s  = std::sqrt(std::sqrt(t * t - ln / a) - t);
    return (x >= 0 ? s : -s);
}
static double chi2invApprox(double p, int dof) {
    const double z = std::sqrt(2.0) * erfinvApprox(2.0 * p - 1.0);
    const double k = static_cast<double>(dof);
    return k * std::pow(1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k)), 3.0);
}

static Eigen::Vector2d predictZ(const Eigen::Vector3d& pose, const Eigen::Vector2d& lm) {
    const double dx = lm(0) - pose(0);
    const double dy = lm(1) - pose(1);
    const double r  = std::hypot(dx, dy);
    const double b  = wrapToPi(std::atan2(dy, dx) - pose(2));
    return {r, b};
}

static Eigen::MatrixXd jacobianH(const Eigen::Vector3d& pose, const Eigen::Vector2d& lm, int j, int N) {
    const double dx = lm(0) - pose(0);
    const double dy = lm(1) - pose(1);
    const double q  = dx*dx + dy*dy;
    const double r  = std::sqrt(q);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 3 + 2*N);

    H(0,0) = -dx/r;  H(0,1) = -dy/r;
    H(1,0) =  dy/q;  H(1,1) = -dx/q;  H(1,2) = -1.0;

    const int idx = 3 + 2*j;
    H(0,idx+0) = dx/r;   H(0,idx+1) = dy/r;
    H(1,idx+0) = -dy/q;  H(1,idx+1) = dx/q;
    return H;
}

static double nis(const Eigen::Vector2d& nu, const Eigen::Matrix2d& S) {
    return nu.transpose() * S.ldlt().solve(nu);
}

static int associateNN(const Eigen::VectorXd& mu,
                       const Eigen::MatrixXd& P,
                       int N,
                       const Eigen::Vector2d& z,
                       const Eigen::Matrix2d& R,
                       double gateProb = 0.99) {
    const double gamma = chi2invApprox(gateProb, 2);
    int bestJ = -1;
    double bestD2 = std::numeric_limits<double>::infinity();

    Eigen::Vector3d pose = mu.segment<3>(0);

    for (int j = 0; j < N; ++j) {
        Eigen::Vector2d lm = mu.segment<2>(3 + 2*j);
        Eigen::Vector2d zhat = predictZ(pose, lm);

        Eigen::Vector2d nu = z - zhat;
        nu(1) = wrapToPi(nu(1));

        Eigen::MatrixXd H = jacobianH(pose, lm, j, N);
        Eigen::Matrix2d S = (H * P * H.transpose()).eval() + R;

        const double d2 = nis(nu, S);
        if (d2 <= gamma && d2 < bestD2) { bestD2 = d2; bestJ = j; }
    }
    return bestJ; // -1: no association
}

int main() {
    Eigen::Vector3d poseTrue(2.0, 1.0, 0.4);
    std::vector<Eigen::Vector2d> lms = {{5,2},{4,-1.5},{1,4},{7,5},{6,-2}};

    Eigen::Vector3d poseEst = poseTrue + Eigen::Vector3d(0.1,-0.05,0.03);
    std::vector<Eigen::Vector2d> lmsEst;
    for (auto& p : lms) lmsEst.push_back(p + Eigen::Vector2d(0.2,-0.1));

    const int N = (int)lmsEst.size();
    Eigen::VectorXd mu(3 + 2*N);
    mu.segment<3>(0) = poseEst;
    for (int j=0;j<N;++j) mu.segment<2>(3+2*j) = lmsEst[j];

    // demo SPD covariance
    int dim = 3 + 2*N;
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(dim, dim);
    Eigen::MatrixXd P = A*A.transpose();
    P /= P.diagonal().maxCoeff();
    Eigen::VectorXd s(dim); s << 0.2,0.2,0.05, Eigen::VectorXd::Constant(2*N,0.5);
    P = s.asDiagonal()*P*s.asDiagonal();

    double sigmaR = 0.15, sigmaB = 2.0*M_PI/180.0;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = sigmaR*sigmaR; R(1,1) = sigmaB*sigmaB;

    std::vector<Eigen::Vector2d> Z = { predictZ(poseTrue, lms[0]),
                                       predictZ(poseTrue, lms[2]),
                                       predictZ(poseTrue, lms[4]) };

    for (int i=0;i<(int)Z.size();++i) {
        int j = associateNN(mu, P, N, Z[i], R, 0.99);
        std::cout << "meas " << i << " -> landmark " << j << "\n";
    }
    return 0;
}
