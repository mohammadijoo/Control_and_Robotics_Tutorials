#include <iostream>
#include <array>
#include <cmath>

// Simple ring buffer for fixed-size memory
template <typename T, std::size_t W>
class RingBuffer {
public:
    RingBuffer() : idx_(0), full_(false) {
        data_.fill(T());
    }

    void push(const T& value) {
        data_[idx_] = value;
        idx_ = (idx_ + 1) % W;
        if (idx_ == 0) {
            full_ = true;
        }
    }

    // Access i-th most recent element (i = 0 is latest)
    T get(std::size_t i) const {
        if (i >= size()) {
            return data_[0];
        }
        std::size_t pos = (idx_ + W - 1 - i) % W;
        return data_[pos];
    }

    std::size_t size() const {
        return full_ ? W : idx_;
    }

private:
    std::array<T, W> data_;
    std::size_t idx_;
    bool full_;
};

struct StateEstimate {
    double x;      // position
    double v;      // velocity (optional)
};

double stageCost(double x, double u, double x_target) {
    double pos_err = x - x_target;
    return pos_err * pos_err + 0.01 * u * u;
}

// Very simple receding-horizon lookahead with heuristic control sequence
double computeOptimalControl(const StateEstimate& x_hat,
                             const RingBuffer<StateEstimate, 32>& memory,
                             double x_target,
                             std::size_t horizon_steps,
                             double dt) {
    // Here we use a crude heuristic: proportional feedback using
    // an estimate that might depend on memory (not shown in detail).
    (void)memory; // in a real system, use past data to refine estimate
    double Kp = 0.5;
    double u = -Kp * (x_hat.x - x_target);
    // Clip control
    if (u > 1.0) u = 1.0;
    if (u < -1.0) u = -1.0;
    return u;
}

int main() {
    constexpr std::size_t W = 32;
    RingBuffer<StateEstimate, W> mem;

    StateEstimate x_hat{0.0, 0.0};
    double x_target = 10.0;
    double dt = 0.1;
    std::size_t H = 200;

    for (std::size_t t = 0; t < H; ++t) {
        // In practice, x_hat would come from an observer (e.g., Kalman filter)
        // that itself uses the memory buffer 'mem'.
        mem.push(x_hat);

        double u = computeOptimalControl(x_hat, mem, x_target, 20, dt);

        // Simulate simple dynamics x_{t+1} = x_t + dt * u
        x_hat.x += dt * u;

        std::cout << "t=" << t <<
                     " x_hat=" << x_hat.x <<
                     " u=" << u << std::endl;
    }
    return 0;
}
      
