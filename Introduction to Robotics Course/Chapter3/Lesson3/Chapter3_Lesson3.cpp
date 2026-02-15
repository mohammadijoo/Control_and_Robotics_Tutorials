#include <iostream>
#include <cmath>
#include <Eigen/Dense>

double cantileverTipDeflection(double F, double L, double E, double I){
    return F * std::pow(L,3) / (3.0 * E * I);
}

Eigen::Vector3d continuumTipPosition(double kappa, double phi, double L){
    if (std::abs(kappa) < 1e-9){
        return Eigen::Vector3d(0,0,L);
    }
    double x = (1.0/kappa) * (1 - std::cos(kappa*L)) * std::cos(phi);
    double y = (1.0/kappa) * (1 - std::cos(kappa*L)) * std::sin(phi);
    double z = (1.0/kappa) * std::sin(kappa*L);
    return Eigen::Vector3d(x,y,z);
}

int main(){
    double F=10.0, L=0.5, E=70e9, I=2e-10;
    std::cout << "Deflection: " << cantileverTipDeflection(F,L,E,I) << std::endl;

    double kappa=4.0, phi=M_PI/6;
    Eigen::Vector3d p = continuumTipPosition(kappa,phi,L);
    std::cout << "Continuum tip: " << p.transpose() << std::endl;
    return 0;
}
      