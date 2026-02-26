
#include <Eigen/Dense>
#include <vector>

struct LQRResult {
    std::vector<Eigen::MatrixXd> P; // P_k, size N+1
    std::vector<Eigen::MatrixXd> K; // K_k, size N
};

LQRResult finiteHorizonLQR(const Eigen::MatrixXd& Ad,
                           const Eigen::MatrixXd& Bd,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R,
                           int N)
{
    const int n = Ad.rows();
    const int m = Bd.cols();

    LQRResult res;
    res.P.resize(N + 1);
    res.K.resize(N);

    // Terminal cost
    res.P[N] = Q;

    for (int k = N - 1; k >= 0; --k) {
        const Eigen::MatrixXd& Pnext = res.P[k + 1];
        Eigen::MatrixXd S = R + Bd.transpose() * Pnext * Bd;
        Eigen::MatrixXd Kk = S.ldlt().solve(Bd.transpose() * Pnext * Ad);
        res.K[k] = Kk;

        res.P[k] = Q + Ad.transpose() * Pnext * Ad
                     - Ad.transpose() * Pnext * Bd * Kk;
    }

    return res;
}

int main()
{
    double I = 0.5;
    double b = 0.1;
    double dt = 0.002;
    double T  = 2.0;
    int N = static_cast<int>(T / dt);

    Eigen::Matrix2d A;
    A << 0.0, 1.0,
          0.0, -b / I;
    Eigen::Vector2d B;
    B << 0.0, 1.0 / I;

    Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + A * dt;
    Eigen::MatrixXd Bd(2,1);
    Bd.col(0) = B * dt;

    Eigen::Matrix2d Q;
    Q.setZero();
    Q(0,0) = 100.0;
    Q(1,1) = 10.0;

    Eigen::MatrixXd R(1,1);
    R(0,0) = 1.0;

    LQRResult res = finiteHorizonLQR(Ad, Bd, Q, R, N);

    // Example closed-loop simulation
    Eigen::Vector2d x(0.3, 0.0); // initial state
    for (int k = 0; k < N; ++k) {
        Eigen::MatrixXd u = -res.K[k] * x;
        Eigen::Vector2d dx = A * x + B * u(0,0);
        x += dt * dx;
        // store or log x, u as needed
    }
    return 0;
}
