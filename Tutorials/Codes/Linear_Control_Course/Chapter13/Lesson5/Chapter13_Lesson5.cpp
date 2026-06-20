#include <iostream>
#include <vector>
#include <complex>
#include <Eigen/Dense>

struct FRFResult {
    std::complex<double> Ghat;
    double omega;
};

FRFResult estimateFRF_singleSine(const std::vector<double> &u,
                                 const std::vector<double> &y,
                                 double Ts,
                                 double omega,
                                 double U0)
{
    std::size_t N = y.size();
    Eigen::MatrixXd Phi(N, 2);
    Eigen::VectorXd yvec(N);

    for (std::size_t k = 0; k < N; ++k) {
        double tk = k * Ts;
        Phi(k, 0) = std::cos(omega * tk);
        Phi(k, 1) = std::sin(omega * tk);
        yvec(k)   = y[k];
    }

    // Least squares: theta = (Phi^T Phi)^(-1) Phi^T y
    Eigen::Vector2d theta =
        (Phi.transpose() * Phi).ldlt().solve(Phi.transpose() * yvec);

    double Ahat = theta(0);
    double Bhat = theta(1);

    // Output phasor estimate Y(j omega) = Bhat - j Ahat
    std::complex<double> Yhat(Bhat, -Ahat);

    // Input phasor for u(t) = U0 sin(omega t) is U0 / j = -j U0
    std::complex<double> Uhat(0.0, -U0);

    std::complex<double> Ghat = Yhat / Uhat;

    FRFResult res;
    res.Ghat = Ghat;
    res.omega = omega;
    return res;
}

int main()
{
    double Ts = 0.001;
    double omega = 5.0;
    double U0 = 0.1;

    // Example data (in practice, fill from sensors)
    std::vector<double> u;
    std::vector<double> y;
    int N = 5000;
    u.reserve(N);
    y.reserve(N);

    // Here we just create synthetic data: u = U0 sin(omega t), y = 2 u (gain 2)
    for (int k = 0; k < N; ++k) {
        double tk = k * Ts;
        double uk = U0 * std::sin(omega * tk);
        double yk = 2.0 * uk; // ideal gain 2
        u.push_back(uk);
        y.push_back(yk);
    }

    // Discard first half as "transient"
    std::vector<double> u_ss(u.begin() + N / 2, u.end());
    std::vector<double> y_ss(y.begin() + N / 2, y.end());

    FRFResult r = estimateFRF_singleSine(u_ss, y_ss, Ts, omega, U0);
    std::cout << "Estimated |G(j omega)| = " << std::abs(r.Ghat)
              << ", angle = " << std::arg(r.Ghat) << " rad\n";
    return 0;
}
