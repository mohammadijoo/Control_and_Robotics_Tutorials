// Chapter15_Lesson2.java
// Runge-Kutta Methods and Step Size Selection
// Java example: RK4 + adaptive step-doubling controller.
// Related library: Apache Commons Math (DormandPrince54Integrator).

import java.util.ArrayList;
import java.util.List;

public class Chapter15_Lesson2 {

    static class AdaptiveResult {
        List<Double> t = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        List<Double> hHistory = new ArrayList<>();
        int accepted = 0;
        int rejected = 0;
    }

    static double f(double t, double y) {
        return -2.0 * y + Math.sin(t);
    }

    static double yExact(double t) {
        return (6.0 / 5.0) * Math.exp(-2.0 * t) + (2.0 * Math.sin(t) - Math.cos(t)) / 5.0;
    }

    static double rk4Step(double t, double y, double h) {
        double k1 = f(t, y);
        double k2 = f(t + 0.5 * h, y + 0.5 * h * k1);
        double k3 = f(t + 0.5 * h, y + 0.5 * h * k2);
        double k4 = f(t + h, y + h * k3);
        return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

    static double[] integrateFixedRK4(double t0, double tf, double y0, double h) {
        int n = (int)Math.ceil((tf - t0) / h);
        double[] out = new double[n + 1];  // store y values only (grid is uniform)
        out[0] = y0;
        double t = t0;
        double y = y0;
        for (int i = 1; i <= n; i++) {
            double hs = Math.min(h, tf - t);
            y = rk4Step(t, y, hs);
            t += hs;
            out[i] = y;
        }
        return out;
    }

    static AdaptiveResult integrateAdaptiveRK4StepDoubling(
            double t0, double tf, double y0,
            double h0, double atol, double rtol,
            double hMin, double hMax, double safety) {

        AdaptiveResult res = new AdaptiveResult();
        res.t.add(t0);
        res.y.add(y0);

        double t = t0;
        double y = y0;
        double h = h0;

        while (t < tf) {
            h = Math.min(h, tf - t);
            if (h < hMin) {
                throw new RuntimeException("Step size below hMin");
            }

            double yFull = rk4Step(t, y, h);
            double yHalf = rk4Step(t, y, 0.5 * h);
            double yHalf2 = rk4Step(t + 0.5 * h, yHalf, 0.5 * h);

            double errEst = Math.abs(yHalf2 - yFull) / 15.0;
            double scale = atol + rtol * Math.max(Math.abs(y), Math.abs(yHalf2));
            double errNorm = (scale > 0.0) ? errEst / scale : errEst;

            if (errNorm <= 1.0) {
                y = yHalf2 + (yHalf2 - yFull) / 15.0;
                t += h;
                res.t.add(t);
                res.y.add(y);
                res.hHistory.add(h);
                res.accepted++;

                double factor;
                if (errNorm == 0.0) {
                    factor = 2.0;
                } else {
                    factor = safety * Math.pow(1.0 / errNorm, 1.0 / 5.0);
                }
                factor = Math.min(2.0, Math.max(0.2, factor));
                h = Math.min(hMax, factor * h);
            } else {
                res.rejected++;
                double factor = safety * Math.pow(1.0 / Math.max(errNorm, 1e-16), 1.0 / 5.0);
                factor = Math.min(1.0, Math.max(0.1, factor));
                h = Math.max(hMin, factor * h);
            }
        }

        return res;
    }

    static double maxErrorUniformGrid(double t0, double h, double[] ys) {
        double emax = 0.0;
        for (int i = 0; i < ys.length; i++) {
            double t = t0 + i * h;
            emax = Math.max(emax, Math.abs(ys[i] - yExact(t)));
        }
        return emax;
    }

    static double maxErrorAdaptive(AdaptiveResult res) {
        double emax = 0.0;
        for (int i = 0; i < res.t.size(); i++) {
            emax = Math.max(emax, Math.abs(res.y.get(i) - yExact(res.t.get(i))));
        }
        return emax;
    }

    public static void main(String[] args) {
        double t0 = 0.0, tf = 10.0, y0 = 1.0, h = 0.1;

        double[] yFixed = integrateFixedRK4(t0, tf, y0, h);
        AdaptiveResult adaptive = integrateAdaptiveRK4StepDoubling(
                t0, tf, y0, 0.2, 1e-8, 1e-6, 1e-8, 0.5, 0.9);

        System.out.println("Chapter15_Lesson2.java");
        System.out.printf("Fixed RK4 (h=%.3f): steps=%d, max error=%.6e%n",
                h, yFixed.length - 1, maxErrorUniformGrid(t0, h, yFixed));

        System.out.printf("Adaptive RK4 step-doubling: accepted=%d, rejected=%d, max error=%.6e%n",
                adaptive.accepted, adaptive.rejected, maxErrorAdaptive(adaptive));

        if (!adaptive.hHistory.isEmpty()) {
            double hMin = adaptive.hHistory.get(0);
            double hMax = adaptive.hHistory.get(0);
            for (double hs : adaptive.hHistory) {
                hMin = Math.min(hMin, hs);
                hMax = Math.max(hMax, hs);
            }
            System.out.printf("h range=[%.3e, %.3e]%n", hMin, hMax);
        }

        System.out.println("Library note: Apache Commons Math provides "
                + "DormandPrince54Integrator and ClassicalRungeKuttaIntegrator.");
    }
}
