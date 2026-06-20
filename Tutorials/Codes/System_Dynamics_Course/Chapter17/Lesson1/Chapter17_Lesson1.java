/*
 * Chapter17_Lesson1.java
 * Random Variables, Random Processes, and Stationarity Concepts
 * Java implementation for System Dynamics (Chapter 17, Lesson 1)
 */

import java.util.Random;

public class Chapter17_Lesson1 {

    static double mean(double[] x) {
        double s = 0.0;
        for (double v : x) s += v;
        return s / x.length;
    }

    static double variance(double[] x) {
        double m = mean(x);
        double s2 = 0.0;
        for (double v : x) {
            double d = v - m;
            s2 += d * d;
        }
        return s2 / x.length;
    }

    static double gaussian(Random rng, double mu, double sigma) {
        return mu + sigma * rng.nextGaussian();
    }

    public static void main(String[] args) {
        Random rng = new Random(17);

        final double A = 2.0;
        final double omega = 3.0;
        final int Nt = 401;
        final int M = 2000;
        final double t0 = 0.0;
        final double t1 = 4.0;
        final double dt = (t1 - t0) / (Nt - 1);

        double[] t = new double[Nt];
        for (int i = 0; i < Nt; i++) {
            t[i] = t0 + i * dt;
        }

        // Random-phase harmonic process X(t)=A cos(omega t + Theta)
        double[] meanX = new double[Nt];
        double[] meanX2 = new double[Nt];

        for (int m = 0; m < M; m++) {
            double theta = 2.0 * Math.PI * rng.nextDouble();
            for (int i = 0; i < Nt; i++) {
                double x = A * Math.cos(omega * t[i] + theta);
                meanX[i] += x;
                meanX2[i] += x * x;
            }
        }
        for (int i = 0; i < Nt; i++) {
            meanX[i] /= M;
            meanX2[i] /= M;
        }

        System.out.println("Random-phase harmonic process estimates:");
        int[] sel = {0, 100, 200, 400};
        for (int idx : sel) {
            double varX = meanX2[idx] - meanX[idx] * meanX[idx];
            System.out.printf("t=%.2f, mean=%.6f, var=%.6f%n", t[idx], meanX[idx], varX);
        }

        // Correlation estimate at selected lags
        int[] lags = {0, 10, 30, 60};
        for (int L : lags) {
            double sum = 0.0;
            long count = 0L;
            for (int m = 0; m < M; m++) {
                double theta = 2.0 * Math.PI * rng.nextDouble();
                for (int i = 0; i < Nt - L; i++) {
                    double x1 = A * Math.cos(omega * t[i] + theta);
                    double x2 = A * Math.cos(omega * t[i + L] + theta);
                    sum += x1 * x2;
                    count++;
                }
            }
            double tau = L * dt;
            double Rhat = sum / count;
            double Rth = 0.5 * A * A * Math.cos(omega * tau);
            System.out.printf("lag=%d, tau=%.3f, Rhat=%.6f, Rtheory=%.6f%n", L, tau, Rhat, Rth);
        }

        // Nonstationary process Y(t)=alpha t + beta
        double[] meanY = new double[Nt];
        double[] meanY2 = new double[Nt];
        for (int m = 0; m < M; m++) {
            double alpha = gaussian(rng, 0.5, 0.2);
            double beta = gaussian(rng, 0.0, 1.0);
            for (int i = 0; i < Nt; i++) {
                double y = alpha * t[i] + beta;
                meanY[i] += y;
                meanY2[i] += y * y;
            }
        }
        for (int i = 0; i < Nt; i++) {
            meanY[i] /= M;
            meanY2[i] /= M;
        }

        System.out.println("\nNonstationary affine-in-time process estimates:");
        for (int idx : sel) {
            double varY = meanY2[idx] - meanY[idx] * meanY[idx];
            System.out.printf("t=%.2f, mean=%.6f, var=%.6f%n", t[idx], meanY[idx], varY);
        }

        // AR(1) process x[k] = a x[k-1] + w[k], stationary if |a| < 1
        final double a = 0.8;
        final double sigmaW = 1.0;
        final int K = 400;
        final int Mar = 500;

        double[] meanAR = new double[K];
        double[] meanAR2 = new double[K];
        double sigmaX0 = sigmaW / Math.sqrt(1.0 - a * a);

        for (int m = 0; m < Mar; m++) {
            double[] x = new double[K];
            x[0] = gaussian(rng, 0.0, sigmaX0);
            for (int k = 1; k < K; k++) {
                x[k] = a * x[k - 1] + gaussian(rng, 0.0, sigmaW);
            }
            for (int k = 0; k < K; k++) {
                meanAR[k] += x[k];
                meanAR2[k] += x[k] * x[k];
            }
        }
        for (int k = 0; k < K; k++) {
            meanAR[k] /= Mar;
            meanAR2[k] /= Mar;
        }

        System.out.println("\nAR(1) stationary regime estimates:");
        int[] ksel = {0, 100, 200, 399};
        for (int k : ksel) {
            double varAR = meanAR2[k] - meanAR[k] * meanAR[k];
            System.out.printf("k=%d, mean=%.6f, var=%.6f%n", k, meanAR[k], varAR);
        }
        System.out.printf("Theoretical stationary variance = %.6f%n", sigmaW * sigmaW / (1.0 - a * a));
    }
}
