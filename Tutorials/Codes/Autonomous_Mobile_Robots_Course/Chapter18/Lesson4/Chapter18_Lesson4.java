// Chapter18_Lesson4.java
// Weather/Lighting Effects on Perception (Outdoor and Field AMR)

import java.util.Random;

public class Chapter18_Lesson4 {

    static class ScalarKalman {
        double x = 0.0;
        double P = 1.0;
        double Q = 0.02;

        void predict() { P += Q; }

        double update(double z, double R) {
            double S = P + R;
            double K = P / S;
            double innov = z - x;
            x = x + K * innov;
            P = (1.0 - K) * P;
            return (innov * innov) / S;
        }
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static double cameraConfidence(double fog, double lux, double rain) {
        double q = Math.pow(lux, 0.7) * Math.exp(-2.2 * fog) * Math.exp(-1.4 * rain);
        return clamp(q, 0.05, 1.0);
    }

    static double lidarConfidence(double fog, double rain) {
        double q = Math.exp(-1.3 * fog) * Math.exp(-0.6 * rain);
        return clamp(q, 0.10, 1.0);
    }

    static double[] gammaNormalize(double[] sig, double gamma) {
        double[] y = new double[sig.length];
        double mn = Double.POSITIVE_INFINITY;
        double mx = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < sig.length; i++) {
            y[i] = Math.pow(clamp(sig[i], 0.0, 1.0), gamma);
            mn = Math.min(mn, y[i]);
            mx = Math.max(mx, y[i]);
        }
        double d = (mx - mn) + 1e-12;
        for (int i = 0; i < y.length; i++) y[i] = (y[i] - mn) / d;
        return y;
    }

    static double contrast(double[] x) {
        double mn = Double.POSITIVE_INFINITY;
        double mx = Double.NEGATIVE_INFINITY;
        for (double v : x) { mn = Math.min(mn, v); mx = Math.max(mx, v); }
        return (mx - mn) / (mx + mn + 1e-12);
    }

    public static void main(String[] args) {
        Random rng = new Random(11);

        int n = 300;
        double[] clear = new double[n];
        double[] deg = new double[n];
        double beta = 0.08, A = 0.85, exposure = 0.30;

        for (int i = 0; i < n; i++) {
            double x = i / (double)(n - 1);
            double J = 0.20 + (x > 0.18 ? 0.35 : 0.0) + 0.12 * Math.sin(12.0 * Math.PI * x) + (x > 0.60 ? 0.18 : 0.0);
            J = clamp(J, 0.0, 1.0);
            clear[i] = J;
            double depth = 4.0 + 28.0 * x;
            double t = Math.exp(-beta * depth);
            double Ifog = J * t + A * (1.0 - t);
            double sigma = Math.sqrt(Math.max(exposure * Ifog, 1e-6)) * 0.05 + 0.015;
            deg[i] = clamp(exposure * Ifog + sigma * rng.nextGaussian(), 0.0, 1.0);
        }
        double[] enh = gammaNormalize(deg, 0.6);
        System.out.printf("Contrast clear=%.4f degraded=%.4f enhanced=%.4f%n", contrast(clear), contrast(deg), contrast(enh));

        final int T = 220;
        final double Rcam0 = 0.60 * 0.60;
        final double Rlidar0 = 0.25 * 0.25;
        double[] truth = new double[T];
        for (int k = 1; k < T; k++) truth[k] = truth[k-1] + 0.10 + 0.05 * rng.nextGaussian();

        ScalarKalman adapt = new ScalarKalman();
        ScalarKalman fixed = new ScalarKalman();

        double seA = 0.0, seF = 0.0;
        for (int k = 0; k < T; k++) {
            double fog = 0.10 + 0.30 * Math.exp(-0.5 * Math.pow((k - 110) / 22.0, 2.0));
            double rain = 0.03 + 0.22 * Math.exp(-0.5 * Math.pow((k - 170) / 14.0, 2.0));
            double lux = (k < 135) ? 1.0 : 0.25;
            double qc = cameraConfidence(fog, lux, rain);
            double ql = lidarConfidence(fog, rain);

            double zCam = truth[k] + Math.sqrt(Rcam0 / qc) * rng.nextGaussian();
            double zLidar = truth[k] + Math.sqrt(Rlidar0 / ql) * rng.nextGaussian();

            adapt.predict(); fixed.predict();
            adapt.update(zCam, Rcam0 / qc);
            adapt.update(zLidar, Rlidar0 / ql);
            fixed.update(zCam, Rcam0);
            fixed.update(zLidar, Rlidar0);

            seA += Math.pow(adapt.x - truth[k], 2.0);
            seF += Math.pow(fixed.x - truth[k], 2.0);
        }

        System.out.printf("Adaptive RMSE=%.4f%n", Math.sqrt(seA / T));
        System.out.printf("Fixed-R RMSE=%.4f%n", Math.sqrt(seF / T));
    }
}
