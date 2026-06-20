// Chapter17_Lesson5.java
import java.io.PrintWriter;
import java.io.IOException;
import java.util.Random;

public class Chapter17_Lesson5 {
    static class State {
        double x;
        double v;
        State(double x, double v) { this.x = x; this.v = v; }
    }

    static double gaussian(Random rng, double mean, double std) {
        return mean + std * rng.nextGaussian();
    }

    static State dynamics(State s, double force, double k, double c, double m) {
        double dx = s.v;
        double dv = -(k / m) * s.x - (c / m) * s.v + force / m;
        return new State(dx, dv);
    }

    static State rk4Step(State s, double force, double k, double c, double m, double h) {
        State k1 = dynamics(s, force, k, c, m);

        State s2 = new State(s.x + 0.5 * h * k1.x, s.v + 0.5 * h * k1.v);
        State k2 = dynamics(s2, force, k, c, m);

        State s3 = new State(s.x + 0.5 * h * k2.x, s.v + 0.5 * h * k2.v);
        State k3 = dynamics(s3, force, k, c, m);

        State s4 = new State(s.x + h * k3.x, s.v + h * k3.v);
        State k4 = dynamics(s4, force, k, c, m);

        double xn = s.x + (h / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
        double vn = s.v + (h / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
        return new State(xn, vn);
    }

    public static void main(String[] args) throws IOException {
        final double T = 10.0;
        final double dt = 0.005;
        final int N = 2000;
        final double m = 1.0;

        final double kMean = 25.0, kStd = 2.0;
        final double cMean = 1.5, cStd = 0.2;
        final double x0Mean = 0.0, x0Std = 0.05;
        final double v0Mean = 0.0, v0Std = 0.05;
        final double sigmaF = 2.0;
        final double xThreshold = 0.75;

        final int nt = (int)Math.round(T / dt) + 1;

        double[] t = new double[nt];
        double[] sumX = new double[nt];
        double[] sumX2 = new double[nt];
        for (int i = 0; i < nt; i++) {
            t[i] = i * dt;
        }

        Random rng = new Random(17L);
        int exceedCount = 0;

        for (int trial = 0; trial < N; trial++) {
            double k = Math.max(1e-6, gaussian(rng, kMean, kStd));
            double c = Math.max(1e-6, gaussian(rng, cMean, cStd));

            State s = new State(gaussian(rng, x0Mean, x0Std), gaussian(rng, v0Mean, v0Std));

            double[] xHist = new double[nt];
            xHist[0] = s.x;

            double peakAbs = Math.abs(s.x);

            for (int n = 0; n < nt - 1; n++) {
                double F = gaussian(rng, 0.0, sigmaF);
                s = rk4Step(s, F, k, c, m, dt);
                xHist[n + 1] = s.x;
                peakAbs = Math.max(peakAbs, Math.abs(s.x));
            }

            for (int n = 0; n < nt; n++) {
                sumX[n] += xHist[n];
                sumX2[n] += xHist[n] * xHist[n];
            }

            if (peakAbs > xThreshold) exceedCount++;
        }

        double[] meanX = new double[nt];
        double[] varX = new double[nt];
        double[] stdX = new double[nt];

        for (int n = 0; n < nt; n++) {
            meanX[n] = sumX[n] / N;
            varX[n] = (sumX2[n] - N * meanX[n] * meanX[n]) / (N - 1.0);
            if (varX[n] < 0.0) varX[n] = 0.0;
            stdX[n] = Math.sqrt(varX[n]);
        }

        final double z975 = 1.959963984540054;
        double seFinal = stdX[nt - 1] / Math.sqrt((double)N);
        double ciMeanLo = meanX[nt - 1] - z975 * seFinal;
        double ciMeanHi = meanX[nt - 1] + z975 * seFinal;

        double pHat = ((double)exceedCount) / N;
        double seP = Math.sqrt(Math.max(pHat * (1.0 - pHat), 1e-12) / N);
        double ciPLo = Math.max(0.0, pHat - z975 * seP);
        double ciPHi = Math.min(1.0, pHat + z975 * seP);

        System.out.printf("Monte Carlo trajectories: %d%n", N);
        System.out.printf("Final mean x(T): %.6f%n", meanX[nt - 1]);
        System.out.printf("Final variance x(T): %.6f%n", varX[nt - 1]);
        System.out.printf("95%% CI for E[x(T)]: [%.6f, %.6f]%n", ciMeanLo, ciMeanHi);
        System.out.printf("P(max |x| > %.3f): %.6f%n", xThreshold, pHat);
        System.out.printf("95%% CI for probability: [%.6f, %.6f]%n", ciPLo, ciPHi);

        try (PrintWriter pw = new PrintWriter("Chapter17_Lesson5_java_results.csv")) {
            pw.println("t,mean_x,var_x,std_x");
            for (int n = 0; n < nt; n++) {
                pw.printf(java.util.Locale.US, "%.6f,%.10f,%.10f,%.10f%n",
                        t[n], meanX[n], varX[n], stdX[n]);
            }
        }
    }
}
