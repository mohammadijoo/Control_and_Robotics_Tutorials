#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

struct Logger {
  std::ofstream out;
  Logger(const std::string& path) : out(path, std::ios::app) {}
  void info(const std::string& msg) { out << "[INFO] " << msg << "\n"; }
  void warn(const std::string& msg) { out << "[WARN] " << msg << "\n"; }
  void error(const std::string& msg){ out << "[ERR ] " << msg << "\n"; }
};

int main() {
  Logger log("robot_run_cpp.log");

  // Scalar residual case
  double S = 0.04;
  double S_inv = 1.0 / S;
  double gamma = 6.635; // chi-square(1, 0.99)

  double xhat1 = 0.0, xhat2 = 0.0;
  double A11 = 1.0, A12 = 0.01, A22 = 1.0;
  double B2 = 0.01;
  double C1 = 1.0;
  double L1 = 0.2, L2 = 2.0;

  for(int k=0; k<100; ++k){
    double u = 0.1;
    double y = 0.0; // measurement
    double cpu = 0.5;

    log.info("k=" + std::to_string(k) +
             " u=" + std::to_string(u) +
             " y=" + std::to_string(y) +
             " cpu=" + std::to_string(cpu));

    double r = y - C1*xhat1;

    // observer
    double xhat1_next = A11*xhat1 + A12*xhat2 + L1*r;
    double xhat2_next = A22*xhat2 + B2*u + L2*r;
    xhat1 = xhat1_next;
    xhat2 = xhat2_next;

    double J = r*r*S_inv;
    if(J > gamma){
      log.error("Fault detected at k=" + std::to_string(k) +
                " J=" + std::to_string(J));
    }
  }
  return 0;
}
      
