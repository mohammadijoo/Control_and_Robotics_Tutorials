
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>

// Replace with your own robot interface
struct JointInterface {
  double readPosition();
  double readVelocity();
  void   writeTorque(double tau);
};

int main() {
  using clock = std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::microseconds;

  JointInterface joint;

  const double Ts = 0.002;   // 2 ms
  const double Kp = 50.0;
  const double Kd = 2.0;

  auto t0 = clock::now();
  auto last = t0;
  auto next = t0 + std::chrono::duration<double>(Ts);

  double e_prev = 0.0;
  double t_final = 5.0;

  std::vector<double> log_t, log_Tk, log_jitter, log_q, log_qd, log_e;

  while (true) {
    auto now = clock::now();
    double t = std::chrono::duration<double>(now - t0).count();
    if (t >= t_final) break;

    double Tk = std::chrono::duration<double>(now - last).count();
    last = now;
    double jitter = Tk - Ts;

    // Joint state
    double q  = joint.readPosition();
    double dq = joint.readVelocity();

    // Reference
    double qd  = 0.5 * std::sin(2.0 * M_PI * 0.5 * t);
    double dqd = 0.5 * 2.0 * M_PI * 0.5 * std::cos(2.0 * M_PI * 0.5 * t);

    double e  = qd - q;
    double de = (e - e_prev) / std::max(Tk, 1e-6);
    e_prev = e;

    double tau = Kp * e + Kd * de;

    joint.writeTorque(tau);

    // Log signals
    log_t.push_back(t);
    log_Tk.push_back(Tk);
    log_jitter.push_back(jitter);
    log_q.push_back(q);
    log_qd.push_back(qd);
    log_e.push_back(e);

    // Sleep until next nominal instant
    next += std::chrono::duration<double>(Ts);
    auto now2 = clock::now();
    if (next > now2) {
      std::this_thread::sleep_until(next);
    }
  }

  // Post-process logs (e.g., save to file) for plotting.
  return 0;
}
