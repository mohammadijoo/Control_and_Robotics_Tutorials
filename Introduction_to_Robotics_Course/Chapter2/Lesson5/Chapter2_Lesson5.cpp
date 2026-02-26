#include <iostream>
#include <Eigen/Dense>

int main() {
    using Eigen::MatrixXd; using Eigen::VectorXd;

    MatrixXd A(2,2); A << 0.9, 0.1, 0.0, 0.85;
    MatrixXd B(2,1); B << 0.1, 0.2;
    MatrixXd K(1,2); K << 1.2, 0.4;  // stable feedback (given)

    VectorXd x(2); x << 1.0, 0.0;
    double alpha = 0.05; // residual bound coefficient

    auto residual = [&](const VectorXd& x){
        // placeholder "learned" residual r_theta(x)
        // must satisfy ||r|| <= alpha ||x|| for safety
        double r = alpha * x.norm(); 
        return r;
    };

    for(int t=0; t<50; ++t){
        double u = -(K*x)(0) + residual(x);
        x = A*x + B*u;
        std::cout << "t=" << t << " x=" << x.transpose() << std::endl;
    }
}
      