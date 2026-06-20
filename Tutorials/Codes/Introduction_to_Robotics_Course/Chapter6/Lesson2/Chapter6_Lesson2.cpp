#include <iostream>
#include <vector>
#include <cmath>

double eq_ratio(const std::vector<std::pair<double,double>>& stages){
    double N = 1.0;
    for(const auto& s : stages){
        double z1 = s.first, z2 = s.second;
        N *= (z2 / z1);
    }
    return N;
}

double backlash_torque(double theta_m, double theta_l, double N,
                       double b, double kt){
    double dtheta = theta_m / N - theta_l;
    if(std::abs(dtheta) <= b/2.0) return 0.0;
    if(dtheta > b/2.0) return kt * (dtheta - b/2.0);
    return kt * (dtheta + b/2.0);
}

int main(){
    std::vector<std::pair<double,double>> stages{
      {20,80},
      {18,54}
    };
    double N = eq_ratio(stages);
    std::cout << "N_eq=" << N << std::endl;

    double tau = backlash_torque(N*0.03, 0.0, N, 0.02, 150.0);
    std::cout << "tau_l=" << tau << std::endl;
    return 0;
}
