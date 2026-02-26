/*
Chapter 8 - Particle-Filter Localization
Lesson 1: Monte Carlo Localization (MCL) Intuition

Minimal from-scratch MCL in a known 2D landmark map (ranges).
Output: prints true and estimated pose each step.

Compile:
  javac Chapter8_Lesson1.java
Run:
  java Chapter8_Lesson1
*/

import java.util.Arrays;
import java.util.Random;

public class Chapter8_Lesson1 {

    static class Particle {
        double x, y, th;
        double w;
        Particle(double x, double y, double th, double w) {
            this.x = x; this.y = y; this.th = th; this.w = w;
        }
    }

    static double wrapAngle(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0 * pi);
        if (a < 0.0) a += 2.0 * pi;
        return a - pi;
    }

    // Box-Muller
    static double randn(Random rng) {
        double u1 = Math.max(1e-12, rng.nextDouble());
        double u2 = rng.nextDouble();
        return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
    }

    static void motionSample(double[] out, double x, double y, double th,
                             double vCmd, double wCmd, double dt,
                             double sigmaV, double sigmaW, Random rng) {
        double v = vCmd + sigmaV * randn(rng);
        double w = wCmd + sigmaW * randn(rng);

        if (Math.abs(w) < 1e-9) {
            out[0] = x + v * dt * Math.cos(th);
            out[1] = y + v * dt * Math.sin(th);
            out[2] = th;
        } else {
            out[0] = x + (v / w) * (Math.sin(th + w * dt) - Math.sin(th));
            out[1] = y + (v / w) * (-Math.cos(th + w * dt) + Math.cos(th));
            out[2] = wrapAngle(th + w * dt);
        }
    }

    static double[] expectedRanges(double x, double y, double[][] landmarks) {
        double[] r = new double[landmarks.length];
        for (int i = 0; i < landmarks.length; i++) {
            double dx = landmarks[i][0] - x;
            double dy = landmarks[i][1] - y;
            r[i] = Math.sqrt(dx*dx + dy*dy);
        }
        return r;
    }

    static double gaussianLogPdf(double[] e, double sigma) {
        double logNorm = -Math.log(sigma * Math.sqrt(2.0 * Math.PI));
        double s = 0.0;
        for (double v : e) s += -0.5 * (v / sigma) * (v / sigma) + logNorm;
        return s;
    }

    static Particle[] systematicResample(Particle[] P, Random rng) {
        int N = P.length;
        double[] cdf = new double[N];
        cdf[0] = P[0].w;
        for (int i = 1; i < N; i++) cdf[i] = cdf[i-1] + P[i].w;
        cdf[N-1] = 1.0;

        double u0 = rng.nextDouble() / (double)N;
        Particle[] out = new Particle[N];
        int i = 0;
        for (int m = 0; m < N; m++) {
            double u = u0 + (double)m / (double)N;
            while (u > cdf[i] && i < N-1) i++;
            out[m] = new Particle(P[i].x, P[i].y, P[i].th, 1.0 / (double)N);
        }
        return out;
    }

    static double[] meanPose(Particle[] P) {
        int N = P.length;
        double mx = 0.0, my = 0.0;
        double s = 0.0, c = 0.0;
        for (Particle p : P) {
            mx += p.x; my += p.y;
            s += Math.sin(p.th);
            c += Math.cos(p.th);
        }
        mx /= (double)N;
        my /= (double)N;
        double mth = Math.atan2(s / (double)N, c / (double)N);
        return new double[]{mx, my, mth};
    }

    public static void main(String[] args) {
        Random rng = new Random(0);

        double[][] landmarks = new double[][]{
                {-5.0, -5.0}, {5.0, -5.0}, {5.0, 5.0}, {-5.0, 5.0}
        };

        double[] xTrue = new double[]{-3.0, -2.0, 0.3};

        int N = 800;
        double sigmaV = 0.10, sigmaW = 0.05, sigmaR = 0.35;
        double dt = 0.1;

        Particle[] P = new Particle[N];
        for (int i = 0; i < N; i++) {
            double x = 3.0 * randn(rng);
            double y = 3.0 * randn(rng);
            double th = -Math.PI + 2.0 * Math.PI * rng.nextDouble();
            P[i] = new Particle(x, y, th, 1.0 / (double)N);
        }

        int T = 120;
        for (int t = 0; t < T; t++) {
            double vCmd = 0.6, wCmd = 0.25;

            // True robot (noise-free for demo)
            double[] tmp = new double[3];
            motionSample(tmp, xTrue[0], xTrue[1], xTrue[2], vCmd, wCmd, dt, 0.0, 0.0, rng);
            xTrue = tmp;

            // Measurement: ranges with noise
            double[] z = expectedRanges(xTrue[0], xTrue[1], landmarks);
            for (int i = 0; i < z.length; i++) z[i] += sigmaR * randn(rng);

            // Predict
            for (Particle p : P) {
                motionSample(tmp, p.x, p.y, p.th, vCmd, wCmd, dt, sigmaV, sigmaW, rng);
                p.x = tmp[0]; p.y = tmp[1]; p.th = tmp[2];
            }

            // Weight update
            double[] logw = new double[N];
            double maxLogw = -1e300;
            for (int i = 0; i < N; i++) {
                double[] zhat = expectedRanges(P[i].x, P[i].y, landmarks);
                double[] e = new double[zhat.length];
                for (int k = 0; k < e.length; k++) e[k] = z[k] - zhat[k];
                logw[i] = gaussianLogPdf(e, sigmaR);
                if (logw[i] > maxLogw) maxLogw = logw[i];
            }
            double sumw = 0.0;
            for (int i = 0; i < N; i++) {
                P[i].w = Math.exp(logw[i] - maxLogw);
                sumw += P[i].w;
            }
            for (int i = 0; i < N; i++) P[i].w /= sumw;

            // Resample
            P = systematicResample(P, rng);

            double[] mu = meanPose(P);
            System.out.printf("t=%d true=(%.3f,%.3f,%.3f) est=(%.3f,%.3f,%.3f)%n",
                    t, xTrue[0], xTrue[1], xTrue[2], mu[0], mu[1], mu[2]);
        }

        System.out.println("Done.");
    }
}
