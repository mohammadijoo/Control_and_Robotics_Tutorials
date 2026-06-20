#include <Eigen/Dense>
#include <iostream>
#include <vector>

int main(){
    double Ts = 0.01;
    double m_nom = 2.0;
    int N = 2000;

    std::vector<double> u(N), v_meas(N);
    // Assume u and v_meas are filled from logs (CSV/ROS bag).
    // For demo, create dummy signals:
    for(int k=0;k<N;k++){
        u[k] = 2.0*std::sin(0.5*k*Ts);
        v_meas[k] = 0.5*std::sin(0.5*k*Ts) + 0.02*((rand()%1000)/1000.0 - 0.5);
    }

    Eigen::VectorXd y(N-1);
    Eigen::MatrixXd Phi(N-1,1);

    for(int k=0;k<N-1;k++){
        double dv = (v_meas[k+1]-v_meas[k])/Ts;
        y(k) = dv - u[k]/m_nom;
        Phi(k,0) = -v_meas[k]/m_nom;
    }

    // LS: minimize ||Phi*b - y||^2
    double b_hat = (Phi.transpose()*Phi).ldlt().solve(Phi.transpose()*y)(0);
    std::cout << "Estimated b = " << b_hat << std::endl;
    return 0;
}
      
