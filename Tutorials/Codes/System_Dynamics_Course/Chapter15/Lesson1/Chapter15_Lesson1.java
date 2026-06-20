// Chapter15_Lesson1.java
// Euler and Improved Euler (Heun) for y' = -2 y + sin(t), y(0)=1

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;

public class Chapter15_Lesson1 {

    static double f(double t, double y) {
        return -2.0 * y + Math.sin(t);
    }

    static double exact(double t) {
        return (2.0 * Math.sin(t) - Math.cos(t)) / 5.0 + (6.0 / 5.0) * Math.exp(-2.0 * t);
    }

    static double[][] euler(double t0, double tf, double y0, double h) {
        int n = (int)Math.round((tf - t0) / h);
        double[] t = new double[n + 1];
        double[] y = new double[n + 1];
        t[0] = t0;
        y[0] = y0;
        for (int k = 0; k < n; k++) {
            t[k + 1] = t[k] + h;
            y[k + 1] = y[k] + h * f(t[k], y[k]);
        }
        return new double[][] {t, y};
    }

    static double[][] improvedEuler(double t0, double tf, double y0, double h) {
        int n = (int)Math.round((tf - t0) / h);
        double[] t = new double[n + 1];
        double[] y = new double[n + 1];
        t[0] = t0;
        y[0] = y0;
        for (int k = 0; k < n; k++) {
            t[k + 1] = t[k] + h;
            double s1 = f(t[k], y[k]);
            double yPred = y[k] + h * s1;
            double s2 = f(t[k + 1], yPred);
            y[k + 1] = y[k] + 0.5 * h * (s1 + s2);
        }
        return new double[][] {t, y};
    }

    static double maxAbsError(double[] t, double[] y) {
        double emax = 0.0;
        for (int k = 0; k < t.length; k++) {
            double e = Math.abs(y[k] - exact(t[k]));
            if (e > emax) emax = e;
        }
        return emax;
    }

    static double estimateOrder(double errH, double errH2) {
        return Math.log(errH / errH2) / Math.log(2.0);
    }

    public static void main(String[] args) throws IOException {
        Locale.setDefault(Locale.US);

        double t0 = 0.0, tf = 5.0, y0 = 1.0;
        double[] hs = {0.2, 0.1, 0.05, 0.025};
        double[] eErr = new double[hs.length];
        double[] hErr = new double[hs.length];

        System.out.println("Step-size study (max error on [0,5])");
        System.out.printf("%-10s %-16s %-16s%n", "h", "Euler", "ImprovedEuler");

        for (int i = 0; i < hs.length; i++) {
            double h = hs[i];
            double[][] outE = euler(t0, tf, y0, h);
            double[][] outH = improvedEuler(t0, tf, y0, h);

            eErr[i] = maxAbsError(outE[0], outE[1]);
            hErr[i] = maxAbsError(outH[0], outH[1]);

            System.out.printf("%-10.3f %-16.8e %-16.8e%n", h, eErr[i], hErr[i]);
        }

        System.out.println("\nEstimated order (successive halving)");
        for (int i = 0; i < hs.length - 1; i++) {
            double pe = estimateOrder(eErr[i], eErr[i + 1]);
            double ph = estimateOrder(hErr[i], hErr[i + 1]);
            System.out.printf("h=%.3f -> %.3f : Euler p~%.4f, Heun p~%.4f%n", hs[i], hs[i+1], pe, ph);
        }

        // Export one trajectory to CSV
        double[][] out = improvedEuler(t0, tf, y0, 0.1);
        try (PrintWriter pw = new PrintWriter(new FileWriter("Chapter15_Lesson1_java_output.csv"))) {
            pw.println("t,heun,exact,abs_error");
            for (int k = 0; k < out[0].length; k++) {
                double t = out[0][k];
                double y = out[1][k];
                double ex = exact(t);
                pw.printf(Locale.US, "%.10f,%.10f,%.10f,%.10f%n", t, y, ex, Math.abs(y - ex));
            }
        }
    }
}
