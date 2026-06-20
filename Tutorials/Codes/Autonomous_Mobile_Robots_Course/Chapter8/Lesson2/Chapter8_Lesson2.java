\
/*
Chapter 8 - Lesson 2: Importance Sampling and Resampling (Particle Filters)

Java implementation (from scratch, standard library only).

Robotics-oriented Java ecosystems (for context):
  - ROSJava (legacy), various academic stacks
  - EJML (matrix computations), though not required here
  - Apache Commons Math (stats), though we implement minimal utilities

Compile:
  javac Chapter8_Lesson2.java
Run:
  java Chapter8_Lesson2
*/

import java.util.Random;

public class Chapter8_Lesson2 {

    static double logSumExp(double[] logw) {
        double m = Double.NEGATIVE_INFINITY;
        for (double v : logw) m = Math.max(m, v);
        double s = 0.0;
        for (double v : logw) s += Math.exp(v - m);
        return m + Math.log(s);
    }

    static double[] normalizeLogWeights(double[] logw) {
        double lse = logSumExp(logw);
        double[] w = new double[logw.length];
        double sum = 0.0;
        for (int i = 0; i < logw.length; i++) {
            w[i] = Math.exp(logw[i] - lse);
            sum += w[i];
        }
        for (int i = 0; i < w.length; i++) w[i] /= sum;
        return w;
    }

    static double effectiveSampleSize(double[] w) {
        double s2 = 0.0;
        for (double wi : w) s2 += wi * wi;
        return 1.0 / s2;
    }

    static int[] systematicResample(double[] w, Random rng) {
        int N = w.length;
        double[] cdf = new double[N];
        double acc = 0.0;
        for (int i = 0; i < N; i++) {
            acc += w[i];
            cdf[i] = acc;
        }

        double u0 = rng.nextDouble() / N;
        int[] a = new int[N];
        int j = 0;
        for (int i = 0; i < N; i++) {
            double u = u0 + ((double) i) / N;
            while (u > cdf[j] && j < N - 1) j++;
            a[i] = j;
        }
        return a;
    }

    static double randn(Random rng) {
        // Box-Muller
        double u1 = Math.max(rng.nextDouble(), 1e-12);
        double u2 = rng.nextDouble();
        return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
    }

    static void demo1DTracking(long seed) {
        Random rng = new Random(seed);

        int N = 2000;
        double q = 0.4; // process std
        double r = 0.7; // measurement std
        double u = 0.3;

        double[] x = new double[N];
        for (int i = 0; i < N; i++) x[i] = 4.0 * randn(rng);

        double xTrue = 2.0;

        for (int t = 1; t <= 8; t++) {
            xTrue = xTrue + u + q * randn(rng);

            for (int i = 0; i < N; i++) x[i] = x[i] + u + q * randn(rng);

            double z = xTrue + r * randn(rng);

            double[] logw = new double[N];
            for (int i = 0; i < N; i++) {
                double e = (z - x[i]) / r;
                logw[i] = -0.5 * e * e;
            }
            double[] w = normalizeLogWeights(logw);

            double Neff = effectiveSampleSize(w);
            double xHat = 0.0;
            for (int i = 0; i < N; i++) xHat += w[i] * x[i];

            System.out.printf("t=%02d, z=%+.3f, true=%+.3f, est=%+.3f, ESS=%.1f%n",
                    t, z, xTrue, xHat, Neff);

            if (Neff < 0.5 * N) {
                int[] a = systematicResample(w, rng);
                double[] xNew = new double[N];
                for (int i = 0; i < N; i++) xNew[i] = x[a[i]];
                x = xNew;
            }
        }
    }

    public static void main(String[] args) {
        demo1DTracking(7L);
    }
}
