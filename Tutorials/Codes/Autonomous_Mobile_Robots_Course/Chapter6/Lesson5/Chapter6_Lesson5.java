// Chapter6_Lesson5.java
// Lab: Build a Basic Bayes Filter Loop (discrete 1D cyclic state)
//
// Compile:
//   javac Chapter6_Lesson5.java
// Run:
//   java Chapter6_Lesson5
//
// This implementation avoids external dependencies; for linear algebra in robotics
// you may consider EJML, Apache Commons Math, or ROSJava bindings.

import java.util.Random;

public class Chapter6_Lesson5 {

    static double wrapInterval(double x, double halfPeriod) {
        double period = 2.0 * halfPeriod;
        double y = (x + halfPeriod) % period;
        if (y < 0.0) y += period;
        return y - halfPeriod;
    }

    static double gaussianPdf(double x, double mu, double sigma) {
        double c = 1.0 / (Math.sqrt(2.0 * Math.PI) * sigma);
        double z = (x - mu) / sigma;
        return c * Math.exp(-0.5 * z * z);
    }

    static double[] buildCyclicKernel(int N, double dx, double deltaM, double sigmaM) {
        double muCells = deltaM / dx;
        double sigmaCells = sigmaM / dx;

        double[] k = new double[N];
        int half = N / 2;
        double sum = 0.0;
        for (int idx = 0; idx < N; idx++) {
            int d = idx;
            if (idx >= half) d = idx - N;
            double v = Math.exp(-0.5 * Math.pow((d - muCells) / sigmaCells, 2.0));
            k[idx] = v;
            sum += v;
        }
        for (int i = 0; i < N; i++) k[i] /= sum;
        return k;
    }

    static double[] predictCyclicDirect(double[] belPrev, double[] kernel) {
        int N = belPrev.length;
        double[] belBar = new double[N];

        for (int j = 0; j < N; j++) {
            double s = 0.0;
            for (int i = 0; i < N; i++) {
                int k = (j - i) % N;
                if (k < 0) k += N;
                s += kernel[k] * belPrev[i];
            }
            belBar[j] = Math.max(0.0, s);
        }

        // normalize
        double sum = 0.0;
        for (double v : belBar) sum += v;
        for (int j = 0; j < N; j++) belBar[j] /= sum;

        return belBar;
    }

    static double[] likelihoodVector(int N, double dx, double landmarkX, double zMeas, double sigmaZ, double halfPeriod) {
        double[] l = new double[N];
        for (int i = 0; i < N; i++) {
            double x = i * dx;
            double d = wrapInterval(x - landmarkX, halfPeriod);
            double v = gaussianPdf(zMeas, d, sigmaZ);
            l[i] = Math.max(v, 1e-300);
        }
        return l;
    }

    static double[] bayesStep(double[] belPrev, double[] kernel,
                             double zMeas, double sigmaZ,
                             double landmarkX, double dx, double halfPeriod) {

        double[] belBar = predictCyclicDirect(belPrev, kernel);
        double[] l = likelihoodVector(belPrev.length, dx, landmarkX, zMeas, sigmaZ, halfPeriod);

        double[] bel = new double[belPrev.length];
        double sum = 0.0;
        for (int i = 0; i < bel.length; i++) {
            bel[i] = belBar[i] * l[i];
            sum += bel[i];
        }
        for (int i = 0; i < bel.length; i++) bel[i] /= sum;
        return bel;
    }

    static int argmax(double[] a) {
        int idx = 0;
        double best = a[0];
        for (int i = 1; i < a.length; i++) {
            if (a[i] > best) {
                best = a[i];
                idx = i;
            }
        }
        return idx;
    }

    public static void main(String[] args) {
        // Discretization
        double L = 10.0;
        int N = 200;
        double dx = L / N;
        double halfPeriod = L / 2.0;

        // Models
        double uDeltaM = 0.35;
        double sigmaM  = 0.12;
        double sigmaZ  = 0.20;
        double landmarkX = 2.0;

        double[] kernel = buildCyclicKernel(N, dx, uDeltaM, sigmaM);

        // Initial belief: uniform
        double[] bel = new double[N];
        for (int i = 0; i < N; i++) bel[i] = 1.0 / N;

        Random rng = new Random(7);
        double xTrue = 7.0;

        int T = 25;
        System.out.println("t, x_true[m], x_hat_MAP[m], z_meas[m]");
        for (int t = 1; t <= T; t++) {
            xTrue = (xTrue + uDeltaM + rng.nextGaussian() * sigmaM) % L;
            if (xTrue < 0.0) xTrue += L;

            double zTrue = wrapInterval(xTrue - landmarkX, halfPeriod);
            double zMeas = wrapInterval(zTrue + rng.nextGaussian() * sigmaZ, halfPeriod);

            bel = bayesStep(bel, kernel, zMeas, sigmaZ, landmarkX, dx, halfPeriod);

            int idxHat = argmax(bel);
            double xHat = idxHat * dx;
            System.out.println(t + ", " + xTrue + ", " + xHat + ", " + zMeas);
        }
    }
}
