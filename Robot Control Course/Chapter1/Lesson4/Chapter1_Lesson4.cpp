
#include <iostream>
#include <Eigen/Dense>

int main() {
    using namespace Eigen;

    // Example: discrete-time joint error dynamics matrix Ad (4x4)
    Matrix4d Ad;
    Ad <<
        0.98, 0.0,  0.01, 0.0,
        0.0,  0.97, 0.0,  0.01,
       -0.5,  0.0,  0.90, 0.0,
        0.0, -0.4,  0.0,  0.88;

    // 1. Spectral radius check
    EigenSolver<Matrix4d> es(Ad);
    std::cout << "Eigenvalues of Ad:\n" << es.eigenvalues() << std::endl;

    // 2. Lyapunov function candidate V(x) = x^T P x
    Matrix4d Q = Matrix4d::Identity();
    Matrix4d P = Matrix4d::Identity(); // here we just choose P = I for illustration

    // Check that V decreases along one time-step for a sample state x
    Vector4d x;
    x << 0.1, -0.1, 0.0, 0.0;

    double Vx  = x.transpose() * P * x;
    Vector4d x_next = Ad * x;
    double Vxnext = x_next.transpose() * P * x_next;

    std::cout << "V(x) = " << Vx << ", V(x_next) = " << Vxnext << std::endl;

    if (Vxnext < Vx) {
        std::cout << "Lyapunov function decreases: system looks stable for this x." << std::endl;
    } else {
        std::cout << "No decrease for this x: adjust Ad or P." << std::endl;
    }

    return 0;
}
