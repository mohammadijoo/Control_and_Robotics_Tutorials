#include <vector>
#include <Eigen/Dense>
#include <cmath>

// Encoder: counts -> angle, velocity
void encoderAngleVelocity(const std::vector<int>& counts,
                          int N, double Ts,
                          std::vector<double>& theta,
                          std::vector<double>& omega) {
    int n = (int)counts.size();
    theta.resize(n);
    omega.resize(n);
    double scale = 2.0 * M_PI / (double)N;

    for (int k = 0; k < n; ++k) {
        theta[k] = scale * (double)counts[k];
        if (k == 0) omega[k] = 0.0;
        else omega[k] = (theta[k] - theta[k-1]) / Ts;
    }
}

// IMU gyro integration (1D)
std::vector<double> integrateGyro(const std::vector<double>& omegaMeas,
                                 double Ts, double theta0 = 0.0) {
    int n = (int)omegaMeas.size();
    std::vector<double> theta(n, 0.0);
    theta[0] = theta0;
    for (int k = 1; k < n; ++k) {
        theta[k] = theta[k-1] + Ts * omegaMeas[k-1];
    }
    return theta;
}

// F/T: voltages -> wrench via least squares
Eigen::VectorXd wrenchFromVoltages(const Eigen::VectorXd& v,
                                   const Eigen::MatrixXd& S_hat) {
    // Solve min ||S_hat w - v||
    return S_hat.colPivHouseholderQr().solve(v);
}
