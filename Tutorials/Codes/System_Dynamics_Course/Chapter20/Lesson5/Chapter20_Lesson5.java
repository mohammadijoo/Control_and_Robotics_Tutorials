// Chapter20_Lesson5.java
/*
System Dynamics — Chapter 20, Lesson 5
Integrated Case Studies and Projects (Mechatronic / Vehicle / Thermal–Fluid)

Self-contained Java 17 code:
- Fixed-step RK4
- Poincaré sampling x(kT) for three periodically forced nonlinear models.

Compile:
  javac Chapter20_Lesson5.java
Run:
  java Chapter20_Lesson5
*/
import java.util.*;
import static java.lang.Math.*;

public class Chapter20_Lesson5 {

  // ---------- RK4 ----------
  interface ODE {
    double[] f(double t, double[] x);
  }
  static double[] rk4Step(ODE ode, double t, double[] x, double dt){
    double[] k1 = ode.f(t, x);
    double[] x2 = new double[x.length];
    for(int i=0;i<x.length;i++) x2[i] = x[i] + 0.5*dt*k1[i];
    double[] k2 = ode.f(t+0.5*dt, x2);
    for(int i=0;i<x.length;i++) x2[i] = x[i] + 0.5*dt*k2[i];
    double[] k3 = ode.f(t+0.5*dt, x2);
    for(int i=0;i<x.length;i++) x2[i] = x[i] + dt*k3[i];
    double[] k4 = ode.f(t+dt, x2);
    double[] xn = new double[x.length];
    for(int i=0;i<x.length;i++){
      xn[i] = x[i] + (dt/6.0)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
    return xn;
  }
  static double[] integrate(ODE ode, double[] x0, double t0, double tf, double dt){
    double t=t0;
    double[] x = Arrays.copyOf(x0, x0.length);
    long steps = (long)ceil((tf - t0)/dt);
    for(long k=0;k<steps;k++){
      x = rk4Step(ode, t, x, dt);
      t += dt;
    }
    return x;
  }

  static List<double[]> poincare(ODE ode, double[] x0, double period,
                                int nTransient, int nPoints, double dt){
    List<double[]> out = new ArrayList<>();
    double t=0.0;
    double[] x = Arrays.copyOf(x0, x0.length);
    for(int k=0;k<nTransient+nPoints;k++){
      x = integrate(ode, x, t, t+period, dt);
      t += period;
      if(k>=nTransient) out.add(Arrays.copyOf(x, x.length));
    }
    return out;
  }

  // ---------- Helpers ----------
  static double sat(double u, double umax){
    return max(-umax, min(umax, u));
  }

  // ---------- Case 1: Motor ----------
  static class MotorParams{
    double J=2e-3, b=1e-3, Kt=0.05, Ke=0.05, R=1.0, L=5e-3;
    double tauC=0.02, tauS=0.03, wS=2.0, uMax=12.0;
  }
  static double stribeck(double w, MotorParams p){
    double s = tanh(50.0*w);
    return (p.tauC + (p.tauS - p.tauC)*exp(-pow(abs(w)/p.wS,2.0))) * s;
  }
  static ODE motorODE(MotorParams p, double amp, double freq){
    // x=[theta, w, i]
    return (t, x) -> {
      double th=x[0], w=x[1], i=x[2];
      double u = sat(amp*sin(2.0*PI*freq*t), p.uMax);
      double tauF = p.b*w + stribeck(w,p);
      double dth = w;
      double dw  = (p.Kt*i - tauF)/p.J;
      double di  = (u - p.R*i - p.Ke*w)/p.L;
      return new double[]{dth,dw,di};
    };
  }

  // ---------- Case 2: Bicycle ----------
  static class VehicleParams{
    double m=1500.0, Iz=2500.0, a=1.2, b=1.6, Ux=20.0;
    double Cf=80000.0, Cr=90000.0, alphaSat=0.15;
    double delta0=0.06, T=1.0;
  }
  static double tire(double alpha, double C, double alphaSat){
    return C*alphaSat*tanh(alpha/alphaSat);
  }
  static ODE bicycleODE(VehicleParams p){
    // x=[vy, r]
    return (t, x) -> {
      double vy=x[0], r=x[1];
      double delta = p.delta0*sin(2.0*PI*t/p.T);
      double alphaF = (vy + p.a*r)/p.Ux - delta;
      double alphaR = (vy - p.b*r)/p.Ux;
      double Fyf = -tire(alphaF, p.Cf, p.alphaSat);
      double Fyr = -tire(alphaR, p.Cr, p.alphaSat);
      double dvy = (Fyf + Fyr)/p.m - p.Ux*r;
      double dr  = (p.a*Fyf - p.b*Fyr)/p.Iz;
      return new double[]{dvy, dr};
    };
  }

  // ---------- Case 3: CSTR ----------
  static class CSTRParams{
    double V=1.0, rho=1000.0, Cp=4180.0, q=1e-3;
    double CAf=1.0, Tf0=300.0, dTf=5.0, T=2.0;
    double k0=7.2e10, E=8.314e4, Rg=8.314;
    double dH=-5e7, UA=5e4, Tc=295.0;
  }
  static ODE cstrODE(CSTRParams p){
    // x=[CA, Temp]
    return (t, x) -> {
      double CA=x[0], Temp=x[1];
      double Tf = p.Tf0 + p.dTf*sin(2.0*PI*t/p.T);
      double k = p.k0*exp(-p.E/(p.Rg*Temp));
      double rA = k*CA;
      double dCA = (p.q/p.V)*(p.CAf - CA) - rA;
      double dT  = (p.q/p.V)*(Tf - Temp)
                 + (-p.dH/(p.rho*p.Cp))*rA
                 - (p.UA/(p.rho*p.Cp*p.V))*(Temp - p.Tc);
      return new double[]{dCA, dT};
    };
  }

  public static void main(String[] args){
    System.out.printf(java.util.Locale.US, "Chapter 20, Lesson 5 — Poincare samples%n");

    // Motor
    MotorParams mp = new MotorParams();
    ODE f1 = motorODE(mp, 8.0, 0.8);
    double T1 = 1.25;
    var P1 = poincare(f1, new double[]{0.0,0.0,0.0}, T1, 50, 5, 1e-4);
    System.out.println("[Motor] theta, w, i:");
    for(double[] s: P1) System.out.printf(java.util.Locale.US, "  %.6f  %.6f  %.6f%n", s[0], s[1], s[2]);

    // Vehicle
    VehicleParams vp = new VehicleParams();
    ODE f2 = bicycleODE(vp);
    var P2 = poincare(f2, new double[]{0.0,0.0}, vp.T, 200, 5, 1e-4);
    System.out.println("[Vehicle] vy, r:");
    for(double[] s: P2) System.out.printf(java.util.Locale.US, "  %.6f  %.6f%n", s[0], s[1]);

    // CSTR
    CSTRParams cp = new CSTRParams();
    ODE f3 = cstrODE(cp);
    var P3 = poincare(f3, new double[]{0.9,305.0}, cp.T, 400, 5, 2e-4);
    System.out.println("[CSTR] CA, T:");
    for(double[] s: P3) System.out.printf(java.util.Locale.US, "  %.6f  %.6f%n", s[0], s[1]);
  }
}
