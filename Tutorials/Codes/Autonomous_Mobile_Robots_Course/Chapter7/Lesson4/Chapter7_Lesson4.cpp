/* 
Chapter7_Lesson4.cpp
Kalman-Filter Localization for AMR — Lesson 4
EKF tuning via innovation statistics (NIS) for a unicycle + GPS example.

Build (example):
  g++ -O2 -std=c++17 Chapter7_Lesson4.cpp -I /usr/include/eigen3 -o ekf_tuning

Dependencies:
  Eigen (header-only)
*/

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat32 = Eigen::Matrix<double,3,2>;
using Mat23 = Eigen::Matrix<double,2,3>;

static double wrap_angle(double a) {
    while (a > M_PI)  a -= 2.0*M_PI;
    while (a <= -M_PI) a += 2.0*M_PI;
    return a;
}

struct NoiseParams {
    double sigma_v;     // process noise on v
    double sigma_w;     // process noise on w
    double sigma_gps;   // measurement noise
    double q_scale = 1.0;
    double r_scale = 1.0;
};

static Vec3 f_unicycle(const Vec3& x, const Vec2& u, double dt) {
    const double px = x(0), py = x(1), th = x(2);
    const double v = u(0), w = u(1);
    Vec3 x2;
    x2(0) = px + v*dt*std::cos(th);
    x2(1) = py + v*dt*std::sin(th);
    x2(2) = wrap_angle(th + w*dt);
    return x2;
}

static Mat3 jacobian_F(const Vec3& x, const Vec2& u, double dt) {
    Mat3 F = Mat3::Identity();
    const double th = x(2);
    const double v = u(0);
    F(0,2) = -v*dt*std::sin(th);
    F(1,2) =  v*dt*std::cos(th);
    return F;
}

static Mat32 jacobian_L(const Vec3& x, double dt) {
    Mat32 L; L.setZero();
    const double th = x(2);
    L(0,0) = dt*std::cos(th);
    L(1,0) = dt*std::sin(th);
    L(2,1) = dt;
    return L;
}

static Mat3 make_Q(const Vec3& x, const Vec2& u, double dt, const NoiseParams& p) {
    Mat32 L = jacobian_L(x, dt);
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
    M(0,0) = p.sigma_v*p.sigma_v;
    M(1,1) = p.sigma_w*p.sigma_w;
    Mat3 Q = p.q_scale * (L * M * L.transpose());
    (void)u; // u not used here (kept for symmetry with theory)
    return Q;
}

static Vec2 h_gps(const Vec3& x) {
    return Vec2(x(0), x(1));
}

static Mat23 jacobian_H() {
    Mat23 H; H.setZero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;
    return H;
}

static Mat2 make_R(const NoiseParams& p) {
    Mat2 R = Mat2::Zero();
    R(0,0) = p.sigma_gps*p.sigma_gps;
    R(1,1) = p.sigma_gps*p.sigma_gps;
    return p.r_scale * R;
}

static double nis(const Vec2& innov, const Mat2& S) {
    return innov.transpose() * S.inverse() * innov;
}

struct SimData {
    std::vector<Vec3> x_true;
    std::vector<Vec2> u_meas;
    std::vector<Vec2> z_gps;
};

static SimData simulate(double T, double dt, unsigned seed) {
    const int N = static_cast<int>(T/dt);
    SimData d;
    d.x_true.resize(N);
    d.u_meas.resize(N);
    d.z_gps.resize(N);

    std::mt19937 gen(seed);
    std::normal_distribution<double> n01(0.0, 1.0);

    // True controls
    std::vector<Vec2> u_true(N);
    for (int k=0; k<N; ++k) {
        const double t = k*dt;
        const double v = 1.0 + 0.2*std::sin(0.2*t);
        const double w = 0.2*std::sin(0.1*t);
        u_true[k] = Vec2(v,w);
        if (k>0) d.x_true[k] = f_unicycle(d.x_true[k-1], u_true[k-1], dt);
    }

    const double sigma_v_meas = 0.08;
    const double sigma_w_meas = 0.04;
    const double sigma_gps_meas = 0.6;

    for (int k=0; k<N; ++k) {
        d.u_meas[k](0) = u_true[k](0) + sigma_v_meas*n01(gen);
        d.u_meas[k](1) = u_true[k](1) + sigma_w_meas*n01(gen);
        d.z_gps[k](0) = d.x_true[k](0) + sigma_gps_meas*n01(gen);
        d.z_gps[k](1) = d.x_true[k](1) + sigma_gps_meas*n01(gen);
    }
    return d;
}

static void run_once(const NoiseParams& p, const SimData& d, double dt,
                     std::vector<double>& nis_list) {
    const int N = static_cast<int>(d.x_true.size());
    Vec3 x(0.0,0.0,0.0);
    Mat3 P = Mat3::Zero();
    P(0,0)=1.0; P(1,1)=1.0; P(2,2)=std::pow(10.0*M_PI/180.0,2);

    Mat23 H = jacobian_H();

    nis_list.clear();
    nis_list.reserve(N);

    for (int k=0; k<N; ++k) {
        // Predict
        Vec3 x_pred = f_unicycle(x, d.u_meas[k], dt);
        Mat3 F = jacobian_F(x, d.u_meas[k], dt);
        Mat3 Q = make_Q(x, d.u_meas[k], dt, p);
        Mat3 P_pred = F*P*F.transpose() + Q;

        // Update (GPS)
        Vec2 y = d.z_gps[k] - h_gps(x_pred);
        Mat2 S = H*P_pred*H.transpose() + make_R(p);
        Eigen::Matrix<double,3,2> K = P_pred*H.transpose()*S.inverse();
        x = x_pred + K*y;
        x(2) = wrap_angle(x(2));
        P = (Mat3::Identity() - K*H)*P_pred;

        nis_list.push_back(nis(y,S));
    }
}

int main() {
    const double dt = 0.1;
    SimData d = simulate(60.0, dt, 2);

    NoiseParams p;
    p.sigma_v = 0.03;
    p.sigma_w = 0.01;
    p.sigma_gps = 0.3;

    std::vector<double> nis_list;

    for (int it=0; it<4; ++it) {
        run_once(p, d, dt, nis_list);
        double mean = 0.0;
        for (double v: nis_list) mean += v;
        mean /= static_cast<double>(nis_list.size());

        std::cout << "Iter " << it << ": r_scale=" << p.r_scale
                  << ", mean NIS=" << mean << " (target ~ 2)" << std::endl;

        // heuristic update
        const double gain = std::min(5.0, std::max(0.2, mean/2.0));
        p.r_scale *= gain;
    }

    std::cout << "Final r_scale=" << p.r_scale << std::endl;
    return 0;
}
