// Chapter20_Lesson4.java
/*
Chapter 20 — Chaos, Complex Dynamics, and Computational Tools
Lesson 4 — Computational Tools

Java demonstration:
- Lorenz ODE simulated with fixed-step RK4
- Simple Poincaré section detection (x crosses 0 with positive direction)
- Logistic map bifurcation tail data and Lyapunov exponent for the map
- CSV export

Compile:
  javac Chapter20_Lesson4.java

Run:
  java Chapter20_Lesson4
*/

import java.io.PrintWriter;
import java.io.IOException;

public class Chapter20_Lesson4 {

  static class LorenzParams {
    double sigma = 10.0;
    double rho   = 28.0;
    double beta  = 8.0 / 3.0;
  }

  static double[] lorenzRhs(double[] s, LorenzParams p) {
    double x = s[0], y = s[1], z = s[2];
    return new double[] {
      p.sigma * (y - x),
      x * (p.rho - z) - y,
      x * y - p.beta * z
    };
  }

  static double[] add(double[] a, double[] b) {
    return new double[] { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
  }

  static double[] mul(double s, double[] a) {
    return new double[] { s * a[0], s * a[1], s * a[2] };
  }

  static double[] rk4Step(double[] s, double h, LorenzParams p) {
    double[] k1 = lorenzRhs(s, p);
    double[] k2 = lorenzRhs(add(s, mul(h * 0.5, k1)), p);
    double[] k3 = lorenzRhs(add(s, mul(h * 0.5, k2)), p);
    double[] k4 = lorenzRhs(add(s, mul(h, k3)), p);

    double[] incr = mul(h / 6.0, add(add(k1, mul(2.0, k2)), add(mul(2.0, k3), k4)));
    return add(s, incr);
  }

  static double logisticNext(double r, double x) {
    return r * x * (1.0 - x);
  }

  static double logisticLyapunov(double r, double x0, int nTransient, int n) {
    double x = x0;
    for (int i = 0; i < nTransient; i++) x = logisticNext(r, x);
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
      x = logisticNext(r, x);
      double d = Math.abs(r * (1.0 - 2.0 * x));
      if (d < 1e-300) d = 1e-300;
      sum += Math.log(d);
    }
    return sum / (double)n;
  }

  public static void main(String[] args) throws IOException {
    String outPrefix = "outputs_ch20_l4_java";

    // --- Lorenz simulation ---
    LorenzParams p = new LorenzParams();
    double[] s = new double[] {1.0, 1.0, 1.0};
    double t = 0.0;
    double T = 40.0;
    double h = 0.001;

    try (PrintWriter traj = new PrintWriter(outPrefix + "_lorenz_traj.csv");
         PrintWriter poinc = new PrintWriter(outPrefix + "_lorenz_poincare_x0.csv")) {

      traj.println("t,x,y,z");
      poinc.println("x,y,z");

      double prevX = s[0];

      while (t <= T) {
        traj.printf("%.10f,%.10f,%.10f,%.10f%n", t, s[0], s[1], s[2]);

        if (prevX < 0.0 && s[0] >= 0.0) {
          poinc.printf("%.10f,%.10f,%.10f%n", s[0], s[1], s[2]);
        }
        prevX = s[0];

        s = rk4Step(s, h, p);
        t += h;
      }
    }

    // --- Logistic map: bifurcation tail data ---
    try (PrintWriter bif = new PrintWriter(outPrefix + "_logistic_bifurcation.csv")) {
      bif.println("r,x");
      double rMin = 2.5, rMax = 4.0;
      int nR = 400, nTrans = 800, nKeep = 120;

      for (int i = 0; i < nR; i++) {
        double r = rMin + (rMax - rMin) * ((double)i / (double)(nR - 1));
        double x = 0.2;
        for (int k = 0; k < nTrans; k++) x = logisticNext(r, x);
        for (int k = 0; k < nKeep; k++) {
          x = logisticNext(r, x);
          bif.printf("%.10f,%.10f%n", r, x);
        }
      }
    }

    // --- Logistic map: Lyapunov curve ---
    try (PrintWriter ly = new PrintWriter(outPrefix + "_logistic_lyapunov.csv")) {
      ly.println("r,lambda");
      int nR2 = 200;
      for (int i = 0; i < nR2; i++) {
        double r = 2.8 + (4.0 - 2.8) * ((double)i / (double)(nR2 - 1));
        double lam = logisticLyapunov(r, 0.2, 1000, 4000);
        ly.printf("%.10f,%.10f%n", r, lam);
      }
    }

    System.out.println("Done. Wrote CSV files with prefix: " + outPrefix + "_*.csv");
  }
}
