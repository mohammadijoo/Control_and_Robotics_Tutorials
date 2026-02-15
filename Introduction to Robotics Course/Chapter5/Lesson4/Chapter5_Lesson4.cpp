
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;

int main() {
    double Ts = 2.0, w0 = 400.0, eta = 0.85;
    double r = 0.6, L = 0.8, E = 70e9, I = 2.5e-7, g = 9.81;

    auto jointTorque = [&](double N, double wj){
        double wm = N*wj;
        double Tm = Ts*(1.0 - wm/w0);
        if (Tm < 0) Tm = 0;
        return eta*N*Tm;
    };

    auto payloadLimit = [&](double N, double wj){
        return jointTorque(N, wj)/(r*g);
    };

    auto stiffness = [&](double E, double I, double L){
        return 3.0*E*I/(L*L*L);
    };

    Vector4d Ns(20,40,80,120);
    for (int i=0;i<Ns.size();++i){
        double N = Ns(i);
        double mp0 = payloadLimit(N, 0.0);
        double wjmax = w0/N;
        std::cout << "N=" << N << " mp_max(0)=" << mp0
                  << " kg, wj_max=" << wjmax << " rad/s\n";
    }

    double k = stiffness(E,I,L);
    std::cout << "k = " << k << " N/m\n";
    return 0;
}
      