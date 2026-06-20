// Chapter18_Lesson5.cpp
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
using Vec3 = std::array<double,3>;

struct P { double L=0.35, Ra=1.8, J=0.02, b=0.08, k=4.0, Kt=0.22; };

double u(double t){ return (t < 0.8) ? 8.0 : ((t < 1.6) ? 3.0 : 0.0); }

Vec3 gradH(const Vec3& x, const P& p){ return {x[0]/p.L, x[1]/p.J, p.k*x[2]}; }

double H(const Vec3& x, const P& p){
    return 0.5*x[0]*x[0]/p.L + 0.5*x[1]*x[1]/p.J + 0.5*p.k*x[2]*x[2];
}

Vec3 f(double t, const Vec3& x, const P& p){
    auto g = gradH(x,p);
    double i=g[0], w=g[1], ks=g[2];
    return {p.Kt*w - p.Ra*i + u(t), -p.Kt*i - ks - p.b*w, w};
}

Vec3 add(const Vec3&a,const Vec3&b,double s){ return {a[0]+s*b[0], a[1]+s*b[1], a[2]+s*b[2]}; }

Vec3 rk4(double t, const Vec3& x, double h, const P& p){
    Vec3 k1=f(t,x,p), k2=f(t+0.5*h, add(x,k1,0.5*h),p), k3=f(t+0.5*h, add(x,k2,0.5*h),p), k4=f(t+h, add(x,k3,h),p);
    return {x[0]+h*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6.0,
            x[1]+h*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6.0,
            x[2]+h*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6.0};
}

int main(){
    P p; double h=1e-3, tf=4.0; int N=int(tf/h)+1; Vec3 x{0,0,0};
    std::ofstream out("Chapter18_Lesson5_cpp_results.csv");
    out << "t,phi,p,q,H,Pin,Pdiss\n" << std::setprecision(10);
    double intPow=0.0, prev=0.0; bool first=true; double H0=H(x,p);
    for(int n=0; n < N; ++n){
        double t=n*h; auto g=gradH(x,p); double i=g[0], w=g[1];
        double Pin=u(t)*i, Pdiss=p.Ra*i*i + p.b*w*w, Hx=H(x,p), s=Pin-Pdiss;
        if(!first) intPow += 0.5*h*(prev+s); first=false; prev=s;
        out << t << "," << x[0] << "," << x[1] << "," << x[2] << "," << Hx << "," << Pin << "," << Pdiss << "\n";
        if(n < N-1) x = rk4(t,x,h,p);
    }
    std::cout << "Energy residual = " << (H(x,p)-H0-intPow) << "\n";
    return 0;
}
