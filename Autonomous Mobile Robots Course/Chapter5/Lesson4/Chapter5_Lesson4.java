// Chapter5_Lesson4.java
// Autonomous Mobile Robots — Chapter 5 Lesson 4: Practical Odometry Filtering
//
// Demonstrates practical deterministic odometry filtering:
//  - Hampel outlier suppression (median + MAD)
//  - First-order low-pass filtering (IIR)
//  - Complementary heading fusion: wheel heading (low-freq) + gyro integration (high-freq)
//  - Planar pose integration
//
// Notes on robotics integration:
//  - In ROS ecosystems, Java integrations exist (e.g., rosjava) but are less common in AMR stacks.
//  - This code is written to be library-light. If you want matrix utilities, EJML is a good choice.
//
// Compile:
//   javac Chapter5_Lesson4.java
// Run:
//   java Chapter5_Lesson4
//
// Output:
//   Writes CSV "chapter5_lesson4_out_java.csv".

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Random;

public class Chapter5_Lesson4 {

  static double wrapToPi(double a) {
    a = (a + Math.PI) % (2.0 * Math.PI);
    if (a < 0) a += 2.0 * Math.PI;
    return a - Math.PI;
  }

  static class FirstOrderLowPass {
    final double alpha;
    double y = 0.0;
    boolean initialized = false;

    FirstOrderLowPass(double alpha) { this.alpha = alpha; }

    static FirstOrderLowPass fromCutoff(double fcHz, double dt) {
      double tau = 1.0 / (2.0 * Math.PI * fcHz);
      double alpha = Math.exp(-dt / tau);
      return new FirstOrderLowPass(alpha);
    }

    double step(double x) {
      if (!initialized) {
        y = x;
        initialized = true;
        return y;
      }
      y = alpha * y + (1.0 - alpha) * x;
      return y;
    }
  }

  static double median(double[] w) {
    double[] c = w.clone();
    Arrays.sort(c);
    return c[c.length / 2];
  }

  static double mad(double[] w, double med) {
    double[] d = new double[w.length];
    for (int i = 0; i < w.length; i++) d[i] = Math.abs(w[i] - med);
    return median(d);
  }

  static double[] hampelFilter(double[] x, int window, double nSigmas) {
    if (window % 2 == 0) throw new IllegalArgumentException("window must be odd");
    int k = window / 2;
    double[] y = x.clone();

    for (int i = k; i < x.length - k; i++) {
      double[] w = new double[window];
      for (int j = 0; j < window; j++) w[j] = x[i - k + j];

      double med = median(w);
      double MAD = mad(w, med) + 1e-12;
      double sigma = 1.4826 * MAD;
      if (Math.abs(x[i] - med) > nSigmas * sigma) y[i] = med;
    }
    return y;
  }

  static class ComplementaryHeadingFilter {
    final double alpha;
    double thetaHat = 0.0;
    boolean initialized = false;

    ComplementaryHeadingFilter(double alpha) { this.alpha = alpha; }

    static ComplementaryHeadingFilter fromCutoff(double fcHz, double dt) {
      double tau = 1.0 / (2.0 * Math.PI * fcHz);
      double alpha = Math.exp(-dt / tau);
      return new ComplementaryHeadingFilter(alpha);
    }

    double step(double omegaGyro, double thetaWheel, double dt, double alphaEff) {
      if (!initialized) {
        thetaHat = thetaWheel;
        initialized = true;
        return thetaHat;
      }
      double pred = wrapToPi(thetaHat + dt * omegaGyro);
      thetaHat = wrapToPi(alphaEff * pred + (1.0 - alphaEff) * thetaWheel);
      return thetaHat;
    }
  }

  static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  public static void main(String[] args) throws IOException {
    double dt = 0.01;
    double T = 20.0;
    int N = (int)(T / dt);

    double[] t = new double[N];
    double[] vTrue = new double[N];
    double[] wTrue = new double[N];
    double[] thetaTrue = new double[N];

    for (int k = 0; k < N; k++) {
      t[k] = k * dt;
      if (t[k] >= 1.0 && t[k] < 6.0) { vTrue[k] = 1.1; wTrue[k] = 0.0; }
      else if (t[k] >= 6.0 && t[k] < 12.0) { vTrue[k] = 0.8; wTrue[k] = 0.35; }
      else if (t[k] >= 12.0 && t[k] < 16.0) { vTrue[k] = 0.0; wTrue[k] = -0.6; }
      else if (t[k] >= 16.0 && t[k] < 20.0) { vTrue[k] = 1.0; wTrue[k] = 0.15; }
      else { vTrue[k] = 0.0; wTrue[k] = 0.0; }
    }

    for (int k = 1; k < N; k++) thetaTrue[k] = wrapToPi(thetaTrue[k - 1] + dt * wTrue[k]);

    Random rng = new Random(2);
    // Wheel measurements: noise + quantization + spikes
    double[] vW = new double[N];
    double[] wW = new double[N];
    double qv = 0.02, qw = 0.02;

    for (int k = 0; k < N; k++) {
      vW[k] = vTrue[k] + 0.05 * rng.nextGaussian();
      wW[k] = wTrue[k] + 0.08 * rng.nextGaussian();
      vW[k] = Math.rint(vW[k] / qv) * qv;
      wW[k] = Math.rint(wW[k] / qw) * qw;
    }

    for (int s = 0; s < 12; s++) {
      int idx = rng.nextInt(N);
      wW[idx] += 2.0 * rng.nextGaussian();
      vW[idx] += 0.8 * rng.nextGaussian();
    }

    // Wheel heading from integrating wheel yaw rate
    double[] thetaW = new double[N];
    for (int k = 1; k < N; k++) thetaW[k] = wrapToPi(thetaW[k - 1] + dt * wW[k]);

    // Gyro yaw rate: bias + noise + random walk bias drift
    double[] wG = new double[N];
    double[] bias = new double[N];
    bias[0] = 0.03;
    for (int k = 1; k < N; k++) bias[k] = bias[k - 1] + 0.0005 * rng.nextGaussian();
    for (int k = 0; k < N; k++) wG[k] = wTrue[k] + bias[k] + 0.05 * rng.nextGaussian();

    // Robust pre-filter
    double[] vWh = hampelFilter(vW, 11, 3.0);
    double[] wWh = hampelFilter(wW, 11, 3.0);

    // Filters
    double vFcHz = 5.0, wFcHz = 8.0, thetaFcHz = 0.7;
    double vMax = 2.0, wMax = 3.0, slipGate = 1.2;

    FirstOrderLowPass vLP = FirstOrderLowPass.fromCutoff(vFcHz, dt);
    FirstOrderLowPass wLP = FirstOrderLowPass.fromCutoff(wFcHz, dt);
    ComplementaryHeadingFilter thetaCF = ComplementaryHeadingFilter.fromCutoff(thetaFcHz, dt);

    double x = 0.0, y = 0.0, theta = 0.0;

    FileWriter out = new FileWriter("chapter5_lesson4_out_java.csv");
    out.write("t,x,y,theta,v_f,w_f\n");

    for (int k = 0; k < N; k++) {
      double v = clamp(vWh[k], -vMax, vMax);
      double ww = clamp(wWh[k], -wMax, wMax);
      double wg = clamp(wG[k], -wMax, wMax);

      double resid = Math.abs(wg - ww);
      double alphaEff = (resid > slipGate) ? Math.min(0.995, thetaCF.alpha + 0.15) : thetaCF.alpha;

      double vF = vLP.step(v);
      double wF = wLP.step(ww);

      theta = thetaCF.step(wg, thetaW[k], dt, alphaEff);

      x += dt * vF * Math.cos(theta);
      y += dt * vF * Math.sin(theta);

      out.write(String.format(java.util.Locale.US, "%f,%f,%f,%f,%f,%f\n",
          t[k], x, y, theta, vF, wF));
    }

    out.close();
    System.out.println("Wrote chapter5_lesson4_out_java.csv");
  }
}
