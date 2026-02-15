#include <iostream>
#include <vector>

int main() {
    double m=1.0, b=0.6, k=4.0;
    double h=0.2, T=5.0;
    int N = (int)(T/h);

    std::vector<double> q(N+1,0.0), dq(N+1,0.0);
    auto u = [](double t){ return 1.0; };

    for(int i=0;i<N;i++){
        double ddq = (u(i*h) - b*dq[i] - k*q[i])/m;
        dq[i+1] = dq[i] + h*ddq;
        q[i+1]  = q[i]  + h*dq[i];
    }
    std::cout << "final q = " << q[N] << std::endl;
    return 0;
}
      
