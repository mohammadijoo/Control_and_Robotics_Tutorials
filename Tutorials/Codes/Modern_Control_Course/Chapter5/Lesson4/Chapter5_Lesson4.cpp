#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct StateSpace {
  Eigen::MatrixXd A, B, C, D;
};

StateSpace phaseVariableRealization(const std::vector<double>& a,
                                    const std::vector<double>& b,
                                    const Eigen::MatrixXd& C,
                                    const Eigen::MatrixXd& D) {
  const int n = static_cast<int>(a.size());
  const int m = static_cast<int>(b.size());

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n - 1; ++i) A(i, i + 1) = 1.0;

  // Last row: [-a0, -a1, ..., -a_{n-1}]
  for (int j = 0; j < n; ++j) A(n - 1, j) = -a[j];

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, m);
  for (int j = 0; j < m; ++j) B(n - 1, j) = b[j];

  StateSpace sys;
  sys.A = A; sys.B = B; sys.C = C; sys.D = D;
  return sys;
}

int main() {
  // Example 2: y''' + a2 y'' + a1 y' + a0 y = b1 u1 + b2 u2
  std::vector<double> a = {2.0, 3.0, 1.0};   // a0, a1, a2
  std::vector<double> b = {5.0, -1.0};       // b1, b2

  Eigen::MatrixXd C(2,3);
  C << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0;
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2,2);

  StateSpace sys = phaseVariableRealization(a, b, C, D);

  std::cout << "A=\n" << sys.A << "\n\n";
  std::cout << "B=\n" << sys.B << "\n\n";
  std::cout << "C=\n" << sys.C << "\n\n";
  std::cout << "D=\n" << sys.D << "\n\n";
  return 0;
}
      
