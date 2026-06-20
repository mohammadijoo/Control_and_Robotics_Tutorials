// Minimal 3D rigid-body translational integrator (no rotation)
// Requires Eigen: https://eigen.tuxfamily.org/
#include <iostream>
#include <Eigen/Dense>

struct Body {
  double m;
  Eigen::Vector3d p;  // position
  Eigen::Vector3d v;  // velocity
};

int main() {
  Body b;
  b.m = 1.5;
  b.p = Eigen::Vector3d(0,0,1);
  b.v = Eigen::Vector3d(0,0,0);

  Eigen::Vector3d g(0,0,-9.81);
  double h = 0.001;

  for (int k=0; k<10000; ++k) {
    Eigen::Vector3d f_ext = b.m * g; // gravity only
    Eigen::Vector3d a = f_ext / b.m;

    // semi-implicit Euler
    b.v += h * a;
    b.p += h * b.v;
  }

  std::cout << "p=" << b.p.transpose() << " v=" << b.v.transpose() << std::endl;
  return 0;
}
      
