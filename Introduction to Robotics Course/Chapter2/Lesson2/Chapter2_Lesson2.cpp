#include <iostream>
#include <vector>
#include <algorithm>

double cubic_ptp(double q0, double qf, double T, double t){
    t = std::max(0.0, std::min(t, T));
    double Delta = qf - q0;
    double s = t / T;
    return q0 + 3.0*Delta*s*s - 2.0*Delta*s*s*s;
}

int main(){
    double q0=0.2, qf=1.0, T=2.0;
    for(int i=0;i<5;i++){
        double t = i*(T/4.0);
        std::cout << cubic_ptp(q0,qf,T,t) << std::endl;
    }
    return 0;
}
      