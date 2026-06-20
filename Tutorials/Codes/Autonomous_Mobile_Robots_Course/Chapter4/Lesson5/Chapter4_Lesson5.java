// Chapter4_Lesson5.java
// Parameter Effects on Real Navigation (Mobile Robot Dynamics - Applied)
//
// Plain Java implementation of the parameter-sensitivity simulation.
// No external dependencies are required.
//
// Compile and run:
//   javac Chapter4_Lesson5.java
//   java Chapter4_Lesson5

import java.util.*;

public class Chapter4_Lesson5 {

  static class Params {
    double m  = 25.0;
    double Iz = 2.0;
    double b  = 0.45;
    double r  = 0.10;
    double mu = 0.8;
    double cv = 0.4;
    double cw = 0.6;
    double g  = 9.81;
  }

  static class State {
    double px=0, py=0, th=0, v=0, om=0;
    State copy() {
      State s = new State();
      s.px=px; s.py=py; s.th=th; s.v=v; s.om=om;
      return s;
    }
  }

  static double clamp(double x, double lo, double hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  static void reference(double t, double[] ref) {
    ref[0] = 0.9 + 0.2 * Math.sin(0.2 * t);
    ref[1] = 0.35 + 0.15 * Math.sin(0.17 * t + 0.7);
  }

  static class PIController {
    Params pnom;
    double kvp=180.0, kvi=35.0, kwp=18.0, kwi=4.0;
    double iv=0.0, iw=0.0;
    PIController(Params p) { this.pnom = p; }
    void reset(){ iv=0.0; iw=0.0; }

    void torques(State x, double vref, double omref, double dt, double[] outTau) {
      double ev = vref - x.v;
      double ew = omref - x.om;
      iv += ev * dt;
      iw += ew * dt;

      double Fcmd = pnom.m  * (kvp * ev + kvi * iv) + pnom.m  * pnom.cv * x.v;
      double Mcmd = pnom.Iz * (kwp * ew + kwi * iw) + pnom.Iz * pnom.cw * x.om;

      double tauMax = 35.0;
      double tauR = 0.5 * pnom.r * (Fcmd + 2.0 * Mcmd / pnom.b);
      double tauL = 0.5 * pnom.r * (Fcmd - 2.0 * Mcmd / pnom.b);
      tauR = clamp(tauR, -tauMax, tauMax);
      tauL = clamp(tauL, -tauMax, tauMax);
      outTau[0] = tauL;
      outTau[1] = tauR;
    }
  }

  static State deriv(State x, double tauL, double tauR, Params p) {
    double F  = (tauR + tauL) / p.r;
    double Mz = (p.b / (2.0 * p.r)) * (tauR - tauL);

    double Fmax = p.mu * p.m * p.g;
    F = clamp(F, -Fmax, Fmax);

    State dx = new State();
    dx.px = x.v * Math.cos(x.th);
    dx.py = x.v * Math.sin(x.th);
    dx.th = x.om;
    dx.v  = (1.0 / p.m) * F - p.cv * x.v;
    dx.om = (1.0 / p.Iz) * Mz - p.cw * x.om;
    return dx;
  }

  static State add(State a, State b, double s) {
    State c = a.copy();
    c.px += s*b.px;
    c.py += s*b.py;
    c.th += s*b.th;
    c.v  += s*b.v;
    c.om += s*b.om;
    return c;
  }

  static State rk4Step(State x, double tauL, double tauR, double dt, Params p) {
    State k1 = deriv(x, tauL, tauR, p);
    State k2 = deriv(add(x, k1, 0.5*dt), tauL, tauR, p);
    State k3 = deriv(add(x, k2, 0.5*dt), tauL, tauR, p);
    State k4 = deriv(add(x, k3, dt),     tauL, tauR, p);

    State xn = x.copy();
    xn = add(xn, k1, dt/6.0);
    xn = add(xn, k2, dt/3.0);
    xn = add(xn, k3, dt/3.0);
    xn = add(xn, k4, dt/6.0);
    return xn;
  }

  static double unwrap(double prev, double cur) {
    double d = cur - prev;
    if (d > Math.PI) cur -= 2.0*Math.PI;
    if (d < -Math.PI) cur += 2.0*Math.PI;
    return cur;
  }

  static class Metrics {
    double rmsPos;
    double rmsTh;
  }

  static Metrics simulate(Params pTrue, Params pNom, double T, double dt) {
    int N = (int)(T/dt) + 1;
    PIController ctrl = new PIController(pNom);

    // Baseline
    State[] x0 = new State[N];
    {
      State x = new State();
      double thPrev = 0.0;
      double[] ref = new double[2];
      double[] tau = new double[2];
      for (int i=0;i<N;i++){
        double t=i*dt;
        reference(t, ref);
        ctrl.torques(x, ref[0], ref[1], dt, tau);
        x = rk4Step(x, tau[0], tau[1], dt, pNom);
        x.th = unwrap(thPrev, x.th);
        thPrev = x.th;
        x0[i] = x.copy();
      }
    }

    ctrl.reset();
    State x = new State();
    double thPrev = 0.0;

    double sumPos2=0.0, sumTh2=0.0;
    double[] ref = new double[2];
    double[] tau = new double[2];
    for (int i=0;i<N;i++){
      double t=i*dt;
      reference(t, ref);
      ctrl.torques(x, ref[0], ref[1], dt, tau);
      x = rk4Step(x, tau[0], tau[1], dt, pTrue);
      x.th = unwrap(thPrev, x.th);
      thPrev = x.th;

      double dx = x.px - x0[i].px;
      double dy = x.py - x0[i].py;
      double pe = Math.sqrt(dx*dx + dy*dy);
      double te = x.th - x0[i].th;

      sumPos2 += pe*pe;
      sumTh2  += te*te;
    }

    Metrics m = new Metrics();
    m.rmsPos = Math.sqrt(sumPos2 / N);
    m.rmsTh  = Math.sqrt(sumTh2  / N);
    return m;
  }

  public static void main(String[] args) {
    Params pNom = new Params();
    double T = 35.0, dt = 0.01;

    class SweepItem {
      String name; double[] values; String unit;
      SweepItem(String n, double[] v, String u){ name=n; values=v; unit=u; }
    }
    List<SweepItem> sweep = Arrays.asList(
      new SweepItem("mass_m",      new double[]{15.0,25.0,40.0}, "kg"),
      new SweepItem("yaw_Iz",      new double[]{1.2, 2.0, 3.5},  "kg*m^2"),
      new SweepItem("wheel_r",     new double[]{0.095,0.10,0.105},"m"),
      new SweepItem("wheelbase_b", new double[]{0.40,0.45,0.52}, "m"),
      new SweepItem("friction_mu", new double[]{0.5, 0.8, 1.0},  "-")
    );

    System.out.println("Parameter sweep (controller nominal; plant varies)");
    for (SweepItem item : sweep) {
      for (double v : item.values) {
        Params pTrue = new Params();
        // copy nominal
        pTrue.m=pNom.m; pTrue.Iz=pNom.Iz; pTrue.b=pNom.b; pTrue.r=pNom.r;
        pTrue.mu=pNom.mu; pTrue.cv=pNom.cv; pTrue.cw=pNom.cw; pTrue.g=pNom.g;

        if (item.name.equals("mass_m")) pTrue.m = v;
        if (item.name.equals("yaw_Iz")) pTrue.Iz = v;
        if (item.name.equals("wheel_r")) pTrue.r = v;
        if (item.name.equals("wheelbase_b")) pTrue.b = v;
        if (item.name.equals("friction_mu")) pTrue.mu = v;

        Metrics m = simulate(pTrue, pNom, T, dt);
        System.out.printf("%s=%.3f %s | pos_rms=%.4f m, theta_rms=%.4f rad%n",
          item.name, v, item.unit, m.rmsPos, m.rmsTh);
      }
    }

    // Robotics libraries (for students to explore, not required by this file):
    // - rosjava: ROS 1 Java client
    // - EJML: efficient matrix library for estimation/control
    // - jBullet: physics simulation
  }
}
