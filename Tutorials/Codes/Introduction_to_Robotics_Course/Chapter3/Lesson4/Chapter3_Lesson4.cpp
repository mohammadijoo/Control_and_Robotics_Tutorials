#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;

int main(){
  double g = 9.81, Jtheta = 0.02, dt = 0.01;
  Matrix4d A; A << 0,1,0,0,
                  0,0,g,0,
                  0,0,0,1,
                  0,0,0,0;
  Vector4d B; B << 0,0,0,1.0/Jtheta;

  Matrix4d Ad = (A*dt).exp();                   // Eigen matrix exponential
  Vector4d Bd = A.fullPivLu().solve((Ad-Matrix4d::Identity())*B);

  RowVector4d Kx; Kx << 0.8, 1.2, 6.0, 2.5;

  Vector4d x; x.setZero(); x(0)=1.0;
  for(int k=0;k<2000;k++){
    double u = -(Kx*x)(0);
    x = Ad*x + Bd*u;
  }
  std::cout << "final state:\n" << x << std::endl;
}
      