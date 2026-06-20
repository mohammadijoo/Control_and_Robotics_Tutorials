// Chapter20_Lesson5.cpp
/*
System Dynamics — Chapter 20, Lesson 5
Integrated Case Studies and Projects:
(Mechatronic DC motor, Vehicle bicycle dynamics, Thermal–fluid CSTR)

This file is self-contained (no external ODE libraries):
- Implements fixed-step RK4
- Demonstrates: (i) periodically forced motor model; (ii) periodically forced bicycle model;
  (iii) periodically forced CSTR.
- Outputs a few Poincaré samples x(kT) to stdout.

Compile (example):
  g++ -O2 -std=c++17 Chapter20_Lesson5.cpp -o lesson5

Run:
  ./lesson5
*/
#include <cmath>
#include <iostream>
#include <vector>
#include <array>
#include <iomanip>

static inline double sat(double u, double umax){
  if(u > umax) return umax;
  if(u < -umax) return -umax;
  return u;
}
static inline double tanh_s(double x){ return std::tanh(x); }

// ----------------- RK4 integrator -----------------
template <size_t N, class F>
std::array<double,N> rk4_step(F f, double t, const std::array<double,N>& x, double dt){
  auto k1 = f(t, x);
  std::array<double,N> x2;
  for(size_t i=0;i<N;i++) x2[i] = x[i] + 0.5*dt*k1[i];
  auto k2 = f(t+0.5*dt, x2);
  for(size_t i=0;i<N;i++) x2[i] = x[i] + 0.5*dt*k2[i];
  auto k3 = f(t+0.5*dt, x2);
  for(size_t i=0;i<N;i++) x2[i] = x[i] + dt*k3[i];
  auto k4 = f(t+dt, x2);

  std::array<double,N> xn;
  for(size_t i=0;i<N;i++){
    xn[i] = x[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
  }
  return xn;
}

template <size_t N, class F>
std::array<double,N> integrate(F f, std::array<double,N> x0, double t0, double tf, double dt){
  double t = t0;
  auto x = x0;
  long steps = (long)std::ceil((tf - t0)/dt);
  for(long k=0;k<steps;k++){
    x = rk4_step<N>(f, t, x, dt);
    t += dt;
  }
  return x;
}

// ----------------- Case 1: Motor -----------------
struct MotorParams {
  double J=2e-3, b=1e-3, Kt=0.05, Ke=0.05, R=1.0, L=5e-3;
  double tau_c=0.02, tau_s=0.03, w_s=2.0, u_max=12.0;
};
static inline double stribeck(double w, const MotorParams& p){
  double s = std::tanh(50.0*w);
  double tau = (p.tau_c + (p.tau_s - p.tau_c)*std::exp(-std::pow(std::fabs(w)/p.w_s,2.0))) * s;
  return tau;
}
struct MotorODE {
  MotorParams p;
  double amp=8.0, freq=0.8; // u(t)=amp*sin(2*pi*freq*t)
  std::array<double,3> operator()(double t, const std::array<double,3>& x) const{
    double th=x[0], w=x[1], i=x[2];
    double u = sat(amp*std::sin(2.0*M_PI*freq*t), p.u_max);
    double tau_f = p.b*w + stribeck(w,p);
    double dth = w;
    double dw  = (p.Kt*i - tau_f)/p.J;
    double di  = (u - p.R*i - p.Ke*w)/p.L;
    return {dth,dw,di};
  }
};

// ----------------- Case 2: Bicycle -----------------
struct VehicleParams {
  double m=1500.0, Iz=2500.0, a=1.2, b=1.6, Ux=20.0;
  double Cf=80000.0, Cr=90000.0, alpha_sat=0.15;
  double delta0=0.06, T=1.0;
};
static inline double tire(double alpha, double C, double alpha_sat){
  return C*alpha_sat*std::tanh(alpha/alpha_sat);
}
struct BicycleODE {
  VehicleParams p;
  std::array<double,2> operator()(double t, const std::array<double,2>& x) const{
    double vy=x[0], r=x[1];
    double delta = p.delta0*std::sin(2.0*M_PI*t/p.T);
    double alpha_f = (vy + p.a*r)/p.Ux - delta;
    double alpha_r = (vy - p.b*r)/p.Ux;
    double Fyf = -tire(alpha_f, p.Cf, p.alpha_sat);
    double Fyr = -tire(alpha_r, p.Cr, p.alpha_sat);
    double dvy = (Fyf + Fyr)/p.m - p.Ux*r;
    double dr  = (p.a*Fyf - p.b*Fyr)/p.Iz;
    return {dvy,dr};
  }
};

// ----------------- Case 3: CSTR -----------------
struct CSTRParams {
  double V=1.0, rho=1000.0, Cp=4180.0, q=1e-3;
  double CAf=1.0, Tf0=300.0, dTf=5.0, T=2.0;
  double k0=7.2e10, E=8.314e4, Rg=8.314;
  double dH=-5e7, UA=5e4, Tc=295.0;
};
struct CSTRODE {
  CSTRParams p;
  std::array<double,2> operator()(double t, const std::array<double,2>& x) const{
    double CA=x[0], T=x[1];
    double Tf = p.Tf0 + p.dTf*std::sin(2.0*M_PI*t/p.T);
    double k = p.k0*std::exp(-p.E/(p.Rg*T));
    double rA = k*CA;
    double dCA = (p.q/p.V)*(p.CAf - CA) - rA;
    double dT  = (p.q/p.V)*(Tf - T)
               + (-p.dH/(p.rho*p.Cp))*rA
               - (p.UA/(p.rho*p.Cp*p.V))*(T - p.Tc);
    return {dCA,dT};
  }
};

// ----------------- Poincaré sampling -----------------
template <size_t N, class F>
std::vector<std::array<double,N>> poincare(F f, std::array<double,N> x0, double period,
                                          int n_transient, int n_points, double dt){
  std::vector<std::array<double,N>> out;
  double t=0.0;
  auto x=x0;
  for(int k=0;k<n_transient+n_points;k++){
    x = integrate<N>(f, x, t, t+period, dt);
    t += period;
    if(k>=n_transient) out.push_back(x);
  }
  return out;
}

int main(){
  std::cout << std::setprecision(6) << std::fixed;

  // Motor
  MotorODE f1;
  double T1 = 1.25;
  auto P1 = poincare<3>(f1, {0.0,0.0,0.0}, T1, 50, 5, 1e-4);
  std::cout << "[Motor] Poincare samples (theta,w,i):\n";
  for(auto& s: P1){
    std::cout << "  " << s[0] << "  " << s[1] << "  " << s[2] << "\n";
  }

  // Vehicle
  BicycleODE f2;
  auto P2 = poincare<2>(f2, {0.0,0.0}, f2.p.T, 200, 5, 1e-4);
  std::cout << "[Vehicle] Poincare samples (vy,r):\n";
  for(auto& s: P2){
    std::cout << "  " << s[0] << "  " << s[1] << "\n";
  }

  // CSTR
  CSTRODE f3;
  auto P3 = poincare<2>(f3, {0.9,305.0}, f3.p.T, 400, 5, 2e-4);
  std::cout << "[CSTR] Poincare samples (CA,T):\n";
  for(auto& s: P3){
    std::cout << "  " << s[0] << "  " << s[1] << "\n";
  }
  return 0;
}
