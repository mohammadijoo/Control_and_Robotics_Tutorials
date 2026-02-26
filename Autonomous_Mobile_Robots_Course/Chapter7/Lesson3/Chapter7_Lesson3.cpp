// Chapter7_Lesson3.cpp
// Consistency and Linearization Pitfalls — EKF + NEES/NIS on a 2D AMR localization example.
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter7_Lesson3.cpp -I /usr/include/eigen3 -o lesson7_3
//
// Optional bounds (requires Boost):
//   (most Linux distros already provide boost headers)
//
// This program prints ANEES/ANIS and 95% bounds (if Boost is available).

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Dense>

#ifdef __has_include
#  if __has_include(<boost/math/distributions/chi_squared.hpp>)
#    include <boost/math/distributions/chi_squared.hpp>
#    define HAS_BOOST_CHI2 1
#  endif
#endif

static double wrapAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

static Eigen::Vector3d fMotion(const Eigen::Vector3d& x, const Eigen::Vector2d& u, double dt) {
    const double px = x(0), py = x(1), th = x(2);
    const double v = u(0), om = u(1);
    Eigen::Vector3d xn;
    xn(0) = px + v * dt * std::cos(th);
    xn(1) = py + v * dt * std::sin(th);
    xn(2) = wrapAngle(th + om * dt);
    return xn;
}

static Eigen::Matrix3d jacF(const Eigen::Vector3d& x, const Eigen::Vector2d& u, double dt) {
    const double th = x(2);
    const double v  = u(0);
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * dt * std::sin(th);
    F(1,2) =  v * dt * std::cos(th);
    return F;
}

static Eigen::Vector2d hMeas(const Eigen::Vector3d& x, const Eigen::Vector2d& lm) {
    const double px = x(0), py = x(1), th = x(2);
    const double dx = lm(0) - px;
    const double dy = lm(1) - py;
    const double r  = std::hypot(dx, dy);
    const double b  = wrapAngle(std::atan2(dy, dx) - th);
    return Eigen::Vector2d(r, b);
}

static Eigen::Matrix<double,2,3> jacH(const Eigen::Vector3d& x, const Eigen::Vector2d& lm) {
    const double px = x(0), py = x(1);
    const double dx = lm(0) - px;
    const double dy = lm(1) - py;
    const double r2 = dx*dx + dy*dy;
    const double r  = std::sqrt(std::max(r2, 1e-12));

    Eigen::Matrix<double,2,3> H;
    H.setZero();
    H(0,0) = -dx / r;
    H(0,1) = -dy / r;
    H(1,0) =  dy / r2;
    H(1,1) = -dx / r2;
    H(1,2) = -1.0;
    return H;
}

struct Stats {
    double ane_es = 0.0;
    double an_is  = 0.0;
    int    dof_x  = 3;
    int    dof_z  = 2;
    int    N      = 0;
};

static Stats runMC(double dt, int steps, int trials, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> stdn(0.0, 1.0);

    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0,0) = 0.08*0.08;
    Q(1,1) = 0.08*0.08;
    Q(2,2) = std::pow(3.0*M_PI/180.0, 2);

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = 0.15*0.15;
    R(1,1) = std::pow(2.0*M_PI/180.0, 2);

    auto sampleGaussian = [&](const Eigen::MatrixXd& Cov) {
        Eigen::LLT<Eigen::MatrixXd> llt(Cov);
        Eigen::MatrixXd L = llt.matrixL();
        Eigen::VectorXd z(Cov.rows());
        for (int i=0;i<z.size();++i) z(i) = stdn(rng);
        return L * z;
    };

    std::vector<Eigen::Vector2d> lms = {
        Eigen::Vector2d(5.0, 0.0),
        Eigen::Vector2d(0.0, 5.0),
        Eigen::Vector2d(5.0, 5.0)
    };

    double nees_sum = 0.0;
    double nis_sum  = 0.0;
    int    count    = 0;

    for (int tr=0; tr<trials; ++tr) {
        Eigen::Vector3d x_true(0.0, 0.0, 0.0);
        Eigen::Vector3d x_hat (0.5, -0.4, 10.0*M_PI/180.0);
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
        P(0,0) = 0.8*0.8;
        P(1,1) = 0.8*0.8;
        P(2,2) = std::pow(15.0*M_PI/180.0, 2);

        for (int k=0; k<steps; ++k) {
            const double v  = 1.0 + 0.2*std::sin(0.05*k);
            const double om = 0.35 + 0.25*std::sin(0.03*k);
            Eigen::Vector2d u(v, om);

            // True propagation with noise
            Eigen::Vector3d w = sampleGaussian(Q);
            x_true = fMotion(x_true, u, dt) + w;
            x_true(2) = wrapAngle(x_true(2));

            // Landmark selection
            const Eigen::Vector2d& lm = lms[k % (int)lms.size()];

            // Measurement with noise
            Eigen::Vector2d z = hMeas(x_true, lm) + sampleGaussian(R);
            z(1) = wrapAngle(z(1));

            // EKF predict
            Eigen::Matrix3d F = jacF(x_hat, u, dt);
            Eigen::Vector3d x_pred = fMotion(x_hat, u, dt);
            Eigen::Matrix3d P_pred = F * P * F.transpose() + Q;

            // EKF update
            Eigen::Matrix<double,2,3> H = jacH(x_pred, lm);
            Eigen::Vector2d z_pred = hMeas(x_pred, lm);
            Eigen::Vector2d nu = z - z_pred;
            nu(1) = wrapAngle(nu(1));

            Eigen::Matrix2d S = H * P_pred * H.transpose() + R;
            Eigen::Matrix<double,3,2> K = P_pred * H.transpose() * S.inverse();

            x_hat = x_pred + K * nu;
            x_hat(2) = wrapAngle(x_hat(2));

            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            // Joseph form
            P = (I - K*H) * P_pred * (I - K*H).transpose() + K*R*K.transpose();

            // Stats
            Eigen::Vector3d e = x_true - x_hat;
            e(2) = wrapAngle(e(2));

            const double nees = (e.transpose() * P.inverse() * e)(0);
            const double nis  = (nu.transpose() * S.inverse() * nu)(0);

            nees_sum += nees;
            nis_sum  += nis;
            count++;
        }
    }

    Stats st;
    st.ane_es = nees_sum / (double)count;
    st.an_is  = nis_sum  / (double)count;
    st.N      = count;
    return st;
}

#ifdef HAS_BOOST_CHI2
static std::pair<double,double> chi2BoundsAverage(int dof, int N, double alpha=0.05) {
    using boost::math::chi_squared_distribution;
    chi_squared_distribution<double> dist((double)(N*dof));
    const double lo = boost::math::quantile(dist, alpha/2.0) / (double)N;
    const double hi = boost::math::quantile(dist, 1.0 - alpha/2.0) / (double)N;
    return {lo, hi};
}
#endif

int main() {
    const int steps  = 200;
    const int trials = 40;

    for (auto dt : {0.05, 0.20}) {
        std::cout << "\n=== dt=" << dt << " ===\n";
        Stats st = runMC(dt, steps, trials, 1);

        std::cout << "EKF ANEES=" << st.ane_es << "  (expected ~ " << st.dof_x << ")\n";
        std::cout << "EKF ANIS =" << st.an_is  << "  (expected ~ " << st.dof_z << ")\n";

#ifdef HAS_BOOST_CHI2
        auto neesB = chi2BoundsAverage(st.dof_x, st.N, 0.05);
        auto nisB  = chi2BoundsAverage(st.dof_z, st.N, 0.05);
        std::cout << "95% bounds for ANEES: [" << neesB.first << ", " << neesB.second << "]\n";
        std::cout << "95% bounds for ANIS : [" << nisB.first  << ", " << nisB.second  << "]\n";
#else
        std::cout << "(Boost chi-square not detected; bounds not printed.)\n";
#endif
        std::cout << "Interpretation: ANEES or ANIS systematically above bounds indicates overconfidence.\n";
    }
    return 0;
}
