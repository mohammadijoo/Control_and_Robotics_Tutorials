#include <iostream>
#include <complex>
#include <Eigen/Dense>

int main() {
  using cd = std::complex<double>;
  using Mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using MatC = Eigen::Matrix<cd, Eigen::Dynamic, Eigen::Dynamic>;

  // Example: n=2, SISO
  Mat A(2,2); A << 0.0, 1.0,
                -2.0, -3.0;
  Mat B(2,1); B << 0.0,
                1.0;
  Mat C(1,2); C << 1.0, 0.0;
  Mat D(1,1); D << 0.0;

  cd s(0.0, 2.0); // s = j*2

  MatC I = MatC::Identity(2,2);
  MatC M = s*I - A.cast<cd>();
  MatC X = M.partialPivLu().solve(B.cast<cd>()); // (sI-A)X = B
  MatC G = C.cast<cd>()*X + D.cast<cd>();

  std::cout << "G(j2) = " << G(0,0) << std::endl;
  return 0;
}
      
