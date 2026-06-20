#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

class Watchdog {
public:
  explicit Watchdog(double timeout_s)
    : timeout(std::chrono::duration<double>(timeout_s)),
      last_hb(std::chrono::steady_clock::now()),
      alive(true) {}

  void heartbeat() {
    last_hb.store(std::chrono::steady_clock::now(),
                  std::memory_order_relaxed);
  }

  void start() {
    wd_thread = std::thread([&](){
      while (alive.load()) {
        std::this_thread::sleep_for(timeout / 4);
        auto dt = std::chrono::steady_clock::now() -
                  last_hb.load();
        if (dt > timeout) {
          std::cerr << "[WATCHDOG] timeout; enter safe-stop.\n";
          alive.store(false);
        }
      }
    });
  }

  void stop() {
    alive.store(false);
    if (wd_thread.joinable()) wd_thread.join();
  }

private:
  std::chrono::duration<double> timeout;
  std::atomic<std::chrono::steady_clock::time_point> last_hb;
  std::atomic<bool> alive;
  std::thread wd_thread;
};

// Safety assertion for control saturation
inline void assert_control_limit(double u, double umax) {
  if (std::abs(u) > umax) {
    throw std::runtime_error("Control limit violated");
  }
}
      
