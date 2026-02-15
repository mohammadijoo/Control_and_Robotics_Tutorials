
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

struct State {
  double q;
  double dq;
};

State readState();           // to be implemented for robot or simulator
void sendTorque(double u);   // to be implemented

int main() {
  const double J = 0.05;
  const double b = 0.01;
  const double k_p = 50.0;
  const double k_d = 2.0;

  const double Ts = 0.001;
  const double T_final = 5.0;
  const int N = static_cast<int>(T_final / Ts);

  // Discrete-time error dynamics matrix A_d (forward Euler)
  Matrix2d A_d;
  A_d << 1.0, Ts,
          -Ts * k_p / J, 1.0 - Ts * (k_d + b) / J;

  Vector2d e;  // [position_error, velocity_error]
  e.setZero();
  const double q_ref = 1.0;

  std::ofstream log("log_pd_debug.txt");

  for (int k = 0; k < N; ++k) {
    auto t_start = std::chrono::steady_clock::now();
    double t_k = k * Ts;

    State s = readState();  // q, dq from sensors or simulator
    e(0) = s.q - q_ref;
    e(1) = s.dq;

    double u = -k_p * e(0) - k_d * e(1);

    // Simple saturation
    const double u_min = -5.0;
    const double u_max =  5.0;
    if (u < u_min) u = u_min;
    if (u > u_max) u = u_max;

    sendTorque(u);

    // Logging: time, q, dq, e0, e1, u
    if (log.is_open()) {
      log << t_k << " "
          << s.q << " " << s.dq << " "
          << e(0) << " " << e(1) << " "
          << u << "\n";
    }

    // Next expected sampling time
    auto t_end = t_start + std::chrono::duration<double>(Ts);
    std::this_thread::sleep_until(t_end);
  }

  log.close();

  // Offline: eigenvalue check
  Eigen::EigenSolver<Matrix2d> es(A_d);
  std::cout << "A_d:\n" << A_d << std::endl;
  std::cout << "Eigenvalues:\n" << es.eigenvalues() << std::endl;

  return 0;
}
