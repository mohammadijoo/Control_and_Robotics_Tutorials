#include <Eigen/Dense>
#include <vector>
#include <iostream>

class CollectiveTransport2D {
public:
    CollectiveTransport2D(int N, double kp, double kv,
                          double alpha, double beta,
                          double mass, double dt)
        : N_(N), kp_(kp), kv_(kv),
          alpha_(alpha), beta_(beta),
          m_(mass), dt_(dt)
    {
        c_.setZero();
        v_.setZero();
        goal_ << 1.0, 0.0;
        f_ = Eigen::VectorXd::Ones(N_) / static_cast<double>(N_);
        L_ = ringLaplacian(N_);
    }

    void step() {
        Eigen::Vector2d e = c_ - goal_;
        Eigen::Vector2d Fd = -kp_ * e - kv_ * v_;
        double normFd = Fd.norm();
        if (normFd < 1e-6) return;

        Eigen::Vector2d d = Fd / normFd;
        double Fd_mag = normFd;

        Eigen::VectorXd g = f_ - (Fd_mag / static_cast<double>(N_)) *
                            Eigen::VectorXd::Ones(N_);
        Eigen::VectorXd gdot = -alpha_ * (L_ * g) - beta_ * g;
        f_ += dt_ * gdot;

        for (int i = 0; i < N_; ++i) {
            if (f_(i) < 0.0) f_(i) = 0.0;
            if (f_(i) > 5.0) f_(i) = 5.0;
        }

        Eigen::Vector2d F_net = d * f_.sum();
        Eigen::Vector2d a = F_net / m_;
        v_ += dt_ * a;
        c_ += dt_ * v_;
    }

    const Eigen::Vector2d& position() const { return c_; }

private:
    Eigen::MatrixXd ringLaplacian(int N) {
        Eigen::MatrixXd L = Eigen::MatrixXd::Zero(N, N);
        for (int i = 0; i < N; ++i) {
            L(i, i) = 2.0;
            L(i, (i - 1 + N) % N) = -1.0;
            L(i, (i + 1) % N) = -1.0;
        }
        return L;
    }

    int N_;
    double kp_, kv_, alpha_, beta_, m_, dt_;
    Eigen::Vector2d c_, v_, goal_;
    Eigen::VectorXd f_;
    Eigen::MatrixXd L_;
};

int main() {
    CollectiveTransport2D sim(6, 2.0, 0.5, 1.0, 1.0, 10.0, 0.01);
    for (int k = 0; k < 1000; ++k) {
        sim.step();
    }
    std::cout << "Final position: " << sim.position().transpose() << std::endl;
    return 0;
}
      
