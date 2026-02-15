#include <iostream>
#include <Eigen/Dense>
#include <random>

using Eigen::Vector2d;
using Eigen::MatrixXd;

Vector2d f_motor(const Vector2d& x, double V,
                 double J, double b, double Kt,
                 double Ke, double R, double L) {
    double omega = x(0), i = x(1);
    double domega = (-b/J)*omega + (Kt/J)*i;
    double di     = (-Ke/L)*omega - (R/L)*i + (1.0/L)*V;
    return Vector2d(domega, di);
}

int main() {
    // True parameters
    double Jt=0.02, bt=0.1, Ktt=0.05, Ket=0.05, Rt=2.0, Lt=0.5;
    // Twin parameters
    double Jh=0.03, bh=0.2, Kth=0.04, Keh=Ket, Rh=Rt, Lh=Lt;

    double dt=0.01, T=5.0;
    int N = (int)(T/dt);

    Vector2d x_true(0,0), x_hat(0,0);
    Vector2d Lobs(30.0, 5.0);

    std::default_random_engine rng(0);
    std::normal_distribution<double> noise(0.0, 0.02);

    MatrixXd Phi(N-1, 2);
    Eigen::VectorXd domega(N-1);

    double omega_prev = 0.0;

    for(int k=0;k<N-1;k++){
        double t = k*dt;
        double V = (t>0.5)?6.0:0.0;

        // Physical step
        x_true += dt * f_motor(x_true, V, Jt, bt, Ktt, Ket, Rt, Lt);
        double y_meas = x_true(0) + noise(rng);

        // Twin predict
        x_hat += dt * f_motor(x_hat, V, Jh, bh, Kth, Keh, Rh, Lh);

        // Correct
        double r = y_meas - x_hat(0);
        x_hat += dt * Lobs * r;

        // Build LS regression for domega = -theta1*omega + theta2*i
        if(k>0){
            domega(k-1) = (y_meas - omega_prev)/dt;
            Phi(k-1,0)  = -omega_prev;
            Phi(k-1,1)  = x_hat(1);
        }
        omega_prev = y_meas;
    }

    // Least squares theta = (Phi^T Phi)^-1 Phi^T domega
    MatrixXd A = Phi.transpose()*Phi;
    Eigen::VectorXd bvec = Phi.transpose()*domega;
    Eigen::VectorXd theta = A.ldlt().solve(bvec);

    std::cout << "theta1=b/J estimate: " << theta(0) << std::endl;
    std::cout << "theta2=Kt/J estimate: " << theta(1) << std::endl;
    return 0;
}
      
