// Chapter6_Lesson3.java
// Autonomous Mobile Robots (Control Engineering) - Chapter 6, Lesson 3
// Motion Models vs Sensor Models: A discrete Bayes-filter demonstration (1D grid)
//
// Compile:
//   javac Chapter6_Lesson3.java
// Run:
//   java Chapter6_Lesson3
//
// Notes on libraries:
// - For probability distributions and linear algebra, consider Apache Commons Math and EJML.
// - For robotics middleware integration, ROSJava can bridge into ROS/ROS2 ecosystems.

import java.util.Arrays;

public class Chapter6_Lesson3 {

    static class OneDWorld {
        final int N;
        final double dx;
        final double beaconX;

        OneDWorld(int N, double dx, double beaconX) {
            this.N = N;
            this.dx = dx;
            this.beaconX = beaconX;
        }

        double[] xs() {
            double[] x = new double[N];
            for (int i = 0; i < N; i++) x[i] = i * dx;
            return x;
        }
    }

    static double gaussianPdf(double x, double mu, double sigma) {
        if (sigma <= 0.0) throw new IllegalArgumentException("sigma must be > 0");
        double z = (x - mu) / sigma;
        return (1.0 / (Math.sqrt(2.0 * Math.PI) * sigma)) * Math.exp(-0.5 * z * z);
    }

    static double[] normalize(double[] p) {
        double s = 0.0;
        for (double v : p) s += v;
        if (s <= 1e-15) throw new IllegalStateException("Probability mass nearly zero.");
        double[] out = new double[p.length];
        for (int i = 0; i < p.length; i++) out[i] = p[i] / s;
        return out;
    }

    static double[] motionPredict(double[] bel, double u, double sigmaU, OneDWorld world) {
        double[] xs = world.xs();
        double[] belBar = new double[world.N];
        Arrays.fill(belBar, 0.0);

        // bel_bar(x) = sum_{x'} N(x; x' + u, sigmaU^2) bel(x')
        for (int i = 0; i < world.N; i++) {
            double xPrev = xs[i];
            double mu = xPrev + u;
            for (int j = 0; j < world.N; j++) {
                double x = xs[j];
                belBar[j] += bel[i] * gaussianPdf(x, mu, sigmaU);
            }
        }
        return normalize(belBar);
    }

    static double[] sensorUpdate(double[] belBar, double z, double sigmaZ, OneDWorld world) {
        double[] xs = world.xs();
        double[] post = new double[world.N];

        for (int i = 0; i < world.N; i++) {
            double expected = Math.abs(xs[i] - world.beaconX);
            double likelihood = gaussianPdf(z, expected, sigmaZ);
            post[i] = belBar[i] * likelihood;
        }
        return normalize(post);
    }

    static double entropyNats(double[] p) {
        double H = 0.0;
        for (double pi : p) if (pi > 0.0) H -= pi * Math.log(pi);
        return H;
    }

    static int argMax(double[] p) {
        int idx = 0;
        double best = p[0];
        for (int i = 1; i < p.length; i++) {
            if (p[i] > best) {
                best = p[i];
                idx = i;
            }
        }
        return idx;
    }

    public static void main(String[] args) {
        OneDWorld world = new OneDWorld(121, 0.1, 6.0);

        double[] bel = new double[world.N];
        Arrays.fill(bel, 1.0);
        bel = normalize(bel);

        double[] uSeq = new double[] {0.5, 0.5, 0.5, 0.5};
        double[] zSeq = new double[] {5.5, 5.0, 4.5, 4.0};

        double sigmaU = 0.25;
        double sigmaZ = 0.35;

        System.out.println("t |  MAP estimate (m) |  belief entropy (nats)");
        System.out.println("--+-------------------+----------------------");

        for (int t = 0; t < uSeq.length; t++) {
            double[] belBar = motionPredict(bel, uSeq[t], sigmaU, world);
            bel = sensorUpdate(belBar, zSeq[t], sigmaZ, world);

            double[] xs = world.xs();
            int idx = argMax(bel);
            double xMap = xs[idx];

            System.out.printf("%d | %17.3f | %20.6f%n", (t + 1), xMap, entropyNats(bel));
        }
    }
}
