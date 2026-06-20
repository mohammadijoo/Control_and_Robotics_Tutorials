// Chapter16_Lesson3.cpp
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

using cd = std::complex<double>;

std::vector<double> impulseResponse(const std::vector<double>& b, const std::vector<double>& a, int N){
    std::vector<double> x(N,0.0), y(N,0.0); x[0]=1.0;
    for(int n=0;n<N;++n){
        for(int k=0;k<(int)b.size();++k) if(n-k>=0) y[n] += b[k]*x[n-k];
        for(int k=1;k<(int)a.size();++k) if(n-k>=0) y[n] -= a[k]*y[n-k];
        y[n] /= a[0];
    }
    return y;
}

cd Hejw(const std::vector<double>& b, const std::vector<double>& a, double w){
    cd zinv = std::exp(cd(0.0,-w)), num(0,0), den(0,0);
    for(int k=0;k<(int)b.size();++k) num += b[k]*std::pow(zinv,k);
    for(int k=0;k<(int)a.size();++k) den += a[k]*std::pow(zinv,k);
    return num/den;
}

int main(){
    std::vector<double> b{0.2,0.1}, a{1.0,-1.5,0.56};
    auto h = impulseResponse(b,a,20);
    for(int i=0;i<10;++i) std::cout << "h[" << i << "]=" << h[i] << "\n";
    for(int i=0;i<=4;++i){
        double w = M_PI*i/4.0;
        cd H = Hejw(b,a,w);
        std::cout << "w=" << w << " |H|=" << std::abs(H) << " phase=" << std::arg(H) << "\n";
    }
    return 0;
}
