#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

// ---------------- Driver Layer ----------------
class EncoderDriver {
public:
    EncoderDriver(double counts_per_rev, double alpha=1.0, double beta=0.0)
      : cpr_(counts_per_rev), alpha_(alpha), beta_(beta), y_prev_(0.0), lam_(0.1) {}

    // mock raw read from bus
    int read_raw_counts() {
        static int c = 0;
        return c++ % 4096; // pretend counts arrive from SPI/CAN
    }

    double read_radians() {
        int s_k = read_raw_counts();
        double y_k = alpha_ * s_k + beta_;            // calibration
        y_k = (1.0 - lam_) * y_prev_ + lam_ * y_k;    // filtering
        y_prev_ = y_k;
        // convert counts to radians
        return (2.0 * M_PI * y_k) / cpr_;
    }

private:
    double cpr_, alpha_, beta_;
    double y_prev_, lam_;
};

int main() {
    EncoderDriver enc(4096.0);
    const int Td_ms = 2;

    while(true){
        double theta = enc.read_radians();
        std::cout << "[DRV] theta=" << theta << " rad\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(Td_ms));
    }
}
      
