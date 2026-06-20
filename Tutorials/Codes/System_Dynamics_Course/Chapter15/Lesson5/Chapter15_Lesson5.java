// Chapter15_Lesson5.java
// Numerical Stability, Error Control, and Model Verification via Simulation
// Java implementation for Chapter 15, Lesson 5 (System Dynamics)

import java.util.ArrayList;
import java.util.List;

public class Chapter15_Lesson5 {

    interface RHS {
        double[] eval(double t, double[] y);
    }

    static double[] rk4Step(RHS f, double t, double[] y, double h) {
        double[] k1 = f.eval(t, y);
        double[] y2 = new double[]{y[0] + 0.5*h*k1[0], y[1] + 0.5*h*k1[1]};
        double[] k2 = f.eval(t + 0.5*h, y2);
        double[] y3 = new double[]{y[0] + 0.5*h*k2[0], y[1] + 0.5*h*k2[1]};
        double[] k3 = f.eval(t + 0.5*h, y3);
        double[] y4 = new double[]{y[0] + h*k3[0], y[1] + h*k3[1]};
        double[] k4 = f.eval(t + h, y4);

        return new double[]{
            y[0] + (h/6.0)*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
            y[1] + (h/6.0)*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
        };
    }

    static class AdaptiveResult {
        List<Double> t = new ArrayList<>();
        List<double[]> y = new ArrayList<>();
        int accepted = 0;
        int rejected = 0;
    }

    static AdaptiveResult adaptiveRK4StepDoubling(
            RHS f, double t0, double[] y0, double tf, double h0, double rtol, double atol) {

        int p = 4;
        double t = t0;
        double[] y = new double[]{y0[0], y0[1]};
        double h = h0;
        double hMin = 1e-8;
        double hMax = 0.2;

        AdaptiveResult out = new AdaptiveResult();
        out.t.add(t);
        out.y.add(new double[]{y[0], y[1]});

        while (t < tf) {
            if (t + h > tf) h = tf - t;

            double[] yBig = rk4Step(f, t, y, h);
            double[] yHalf1 = rk4Step(f, t, y, 0.5*h);
            double[] yHalf2 = rk4Step(f, t + 0.5*h, yHalf1, 0.5*h);

            double[] err = new double[2];
            double accum = 0.0;
            for (int i = 0; i < 2; i++) {
                err[i] = (yHalf2[i] - yBig[i]) / (Math.pow(2.0, p) - 1.0);
                double sc = atol + rtol * Math.max(Math.abs(yHalf2[i]), Math.abs(y[i]));
                double z = err[i] / sc;
                accum += z*z;
            }
            double errNorm = Math.sqrt(accum / 2.0);

            if (errNorm <= 1.0) {
                for (int i = 0; i < 2; i++) y[i] = yHalf2[i] + err[i];
                t += h;
                out.t.add(t);
                out.y.add(new double[]{y[0], y[1]});
                out.accepted++;

                double factor = (errNorm == 0.0) ? 2.0 :
                        0.9 * Math.pow(1.0 / errNorm, 1.0 / (p + 1.0));
                factor = Math.max(0.3, Math.min(2.0, factor));
                h = Math.max(hMin, Math.min(hMax, h * factor));
            } else {
                out.rejected++;
                double factor = 0.9 * Math.pow(1.0 / Math.max(errNorm, 1e-16), 1.0 / (p + 1.0));
                factor = Math.max(0.1, Math.min(0.5, factor));
                h = Math.max(hMin, h * factor);
                if (h <= hMin) throw new RuntimeException("Step size underflow.");
            }
        }
        return out;
    }

    static RHS dampedOscillatorRHS(final double omegaN, final double zeta) {
        return (t, x) -> new double[]{
            x[1],
            -2.0 * zeta * omegaN * x[1] - omegaN * omegaN * x[0]
        };
    }

    static double exactQ(double t, double omegaN, double zeta, double q0, double v0) {
        double wd = omegaN * Math.sqrt(1.0 - zeta*zeta);
        double A = q0;
        double B = (v0 + zeta * omegaN * q0) / wd;
        return Math.exp(-zeta * omegaN * t) * (A * Math.cos(wd * t) + B * Math.sin(wd * t));
    }

    static double energy(double[] x, double omegaN) {
        return 0.5 * (x[1]*x[1] + omegaN*omegaN*x[0]*x[0]);
    }

    static void eulerStabilityDemo(double lambda) {
        System.out.println("=== Explicit Euler stability demo ===");
        double[] hs = {0.01, 0.03, 0.05, 0.06};
        for (double h : hs) {
            double amp = Math.abs(1.0 + h * lambda);
            double y = 1.0;
            for (int n = 0; n < 30; n++) y = y + h * lambda * y;
            System.out.printf("h=%.3f, |1+h*lambda|=%.6f, y_N=%e%n", h, amp, y);
        }
        System.out.println();
    }

    static void convergenceDemo() {
        System.out.println("=== RK4 convergence verification ===");
        double omegaN = 4.0, zeta = 0.1, tf = 5.0;
        double[] y0 = {1.0, 0.0};
        RHS f = dampedOscillatorRHS(omegaN, zeta);
        double[] hs = {0.2, 0.1, 0.05, 0.025};
        double[] errs = new double[hs.length];

        for (int i = 0; i < hs.length; i++) {
            double h = hs[i];
            int n = (int)Math.round(tf / h);
            double t = 0.0;
            double[] y = {y0[0], y0[1]};
            double maxErr = 0.0;
            for (int k = 0; k <= n; k++) {
                double qEx = exactQ(t, omegaN, zeta, y0[0], y0[1]);
                maxErr = Math.max(maxErr, Math.abs(y[0] - qEx));
                if (k < n) {
                    y = rk4Step(f, t, y, h);
                    t += h;
                }
            }
            errs[i] = maxErr;
            System.out.printf("h=%.4f, max|q-q_exact|=%e%n", h, maxErr);
        }

        for (int i = 0; i < errs.length - 1; i++) {
            double pObs = Math.log(errs[i] / errs[i + 1]) / Math.log(2.0);
            System.out.printf("p_obs(%.4f->%.4f)=%.4f%n", hs[i], hs[i + 1], pObs);
        }
        System.out.println();
    }

    static void adaptiveDemo() {
        System.out.println("=== Adaptive RK4 (step-doubling) ===");
        double omegaN = 4.0, zeta = 0.05, tf = 12.0;
        double[] y0 = {1.0, 0.0};
        RHS f = dampedOscillatorRHS(omegaN, zeta);

        AdaptiveResult res = adaptiveRK4StepDoubling(f, 0.0, y0, tf, 0.1, 1e-6, 1e-9);

        boolean monotoneEnergy = true;
        double prevE = energy(res.y.get(0), omegaN);
        double maxErr = 0.0;

        for (int i = 1; i < res.t.size(); i++) {
            double Ei = energy(res.y.get(i), omegaN);
            if (Ei - prevE > 1e-8) monotoneEnergy = false;
            prevE = Ei;
        }
        for (int i = 0; i < res.t.size(); i++) {
            double qEx = exactQ(res.t.get(i), omegaN, zeta, y0[0], y0[1]);
            maxErr = Math.max(maxErr, Math.abs(res.y.get(i)[0] - qEx));
        }

        double[] yf = res.y.get(res.y.size() - 1);
        System.out.printf("Accepted=%d, Rejected=%d%n", res.accepted, res.rejected);
        System.out.printf("Final state: q=%f, v=%f%n", yf[0], yf[1]);
        System.out.printf("Energy nonincreasing=%s%n", Boolean.toString(monotoneEnergy));
        System.out.printf("max|q-q_exact| on adaptive mesh=%e%n", maxErr);
    }

    public static void main(String[] args) {
        eulerStabilityDemo(-50.0);
        convergenceDemo();
        adaptiveDemo();
    }
}
