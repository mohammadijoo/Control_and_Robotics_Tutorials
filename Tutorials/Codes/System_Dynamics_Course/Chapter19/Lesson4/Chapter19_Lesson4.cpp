// Chapter19_Lesson4.cpp
// System Dynamics — Chapter 19, Lesson 4
// Discrete-time simulation of a transport delay using:
//   (1) a ring-buffer delay block (exact integer-sample delay)
//   (2) a stable pseudo-distributed N-lag chain (approximation of e^{-sT})
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter19_Lesson4.cpp -o lesson4
//
// Run:
//   ./lesson4

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

class DelayLine {
public:
    explicit DelayLine(int delaySamples)
        : buf_(std::max(1, delaySamples + 1), 0.0), idx_(0) {}

    double step(double u) {
        // read oldest sample (idx_)
        double y = buf_[idx_];
        // write new sample at idx_ (overwrite oldest), then advance
        buf_[idx_] = u;
        idx_ = (idx_ + 1) % static_cast<int>(buf_.size());
        return y;
    }

private:
    std::vector<double> buf_;
    int idx_;
};

class LagChain {
public:
    LagChain(double T, int N)
        : T_(T), N_(N), x_(std::max(1, N), 0.0) {
        if (T_ <= 0.0) throw std::runtime_error("T must be positive.");
        if (N_ < 1) throw std::runtime_error("N must be >= 1.");
    }

    // Continuous-time chain:
    //   x1dot = (N/T) (u - x1)
    //   xkdot = (N/T) (x_{k-1} - xk), k=2..N
    // Output y = xN
    double step(double u, double dt) {
        double alpha = (static_cast<double>(N_) / T_);
        // Euler update (in-place safe using a copy)
        std::vector<double> xnew = x_;
        xnew[0] = x_[0] + dt * alpha * (u - x_[0]);
        for (int k = 1; k < N_; ++k) {
            xnew[k] = x_[k] + dt * alpha * (x_[k - 1] - x_[k]);
        }
        x_.swap(xnew);
        return x_.back();
    }

private:
    double T_;
    int N_;
    std::vector<double> x_;
};

int main() {
    // Simulation settings
    const double dt = 0.001;
    const double tEnd = 8.0;

    // Plant: ydot = -a y + b u_del
    const double a = 1.0, b = 1.0;

    // Transport delay
    const double T = 1.0;
    const int delaySamples = static_cast<int>(std::round(T / dt));

    DelayLine delay(delaySamples);
    LagChain chain(T, 10);

    double y_delay = 0.0;   // output with exact delay (buffer)
    double y_chain = 0.0;   // output with chain approximation

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "# t, y_trueDelay, y_chainApprox\n";

    for (int k = 0; k <= static_cast<int>(tEnd / dt); ++k) {
        double t = k * dt;
        double u = (t >= 0.0) ? 1.0 : 0.0; // unit step

        // exact integer-sample delay of u
        double u_del = delay.step(u);

        // chain approximation produces an approximation to u(t-T)
        double u_chain = chain.step(u, dt);

        // Euler updates
        y_delay = y_delay + dt * (-a * y_delay + b * u_del);
        y_chain = y_chain + dt * (-a * y_chain + b * u_chain);

        if (k % 50 == 0) {
            std::cout << t << ", " << y_delay << ", " << y_chain << "\n";
        }
    }

    std::cout << "\n# Note: The chain approximation is stable and minimum-phase,\n"
              << "# but it does not reproduce an *exact* dead-time unless N→∞.\n";
    return 0;
}
