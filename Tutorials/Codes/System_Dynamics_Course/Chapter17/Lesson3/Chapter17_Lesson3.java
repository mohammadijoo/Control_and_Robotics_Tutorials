// Chapter17_Lesson3.java
// Response of Linear Systems to Random Inputs: Mean and Variance Propagation
// Java implementation: discrete-time mean/variance recursion + Monte Carlo

import java.util.Random;

public class Chapter17_Lesson3 {
    public static void main(String[] args) {
        final double a = 0.93;
        final double b = 0.50;
        final double g = 1.0;
        final double muU = 0.8;
        final double q = 0.25;      // variance of white sequence w_k
        final int K = 150;
        final int M = 30000;

        // Theoretical propagation
        double m = 0.0;
        double P = 0.0;
        for (int k = 0; k < K; k++) {
            m = a * m + b * muU;
            P = a * a * P + g * g * q;
        }

        // Monte Carlo
        Random rng = new Random(1234);
        double[] x = new double[M];
        for (int k = 0; k < K; k++) {
            for (int i = 0; i < M; i++) {
                double wk = Math.sqrt(q) * rng.nextGaussian();
                x[i] = a * x[i] + b * muU + g * wk;
            }
        }

        double meanMC = 0.0;
        for (double xi : x) meanMC += xi;
        meanMC /= M;

        double varMC = 0.0;
        for (double xi : x) {
            double d = xi - meanMC;
            varMC += d * d;
        }
        varMC /= (M - 1);

        double steadyMean = (b * muU) / (1.0 - a);
        double steadyVar = (g * g * q) / (1.0 - a * a);

        System.out.printf("After %d steps:%n", K);
        System.out.printf("Theory mean      = %.6f%n", m);
        System.out.printf("Monte Carlo mean = %.6f%n", meanMC);
        System.out.printf("Theory variance  = %.6f%n", P);
        System.out.printf("Monte Carlo var  = %.6f%n", varMC);
        System.out.printf("Steady mean      = %.6f%n", steadyMean);
        System.out.printf("Steady variance  = %.6f%n", steadyVar);

        // Optional extension: covariance recursion for 2x2 systems
        double[][] A = {{0.88, 0.12}, {-0.15, 0.84}};
        double[][] G = {{0.0}, {1.0}};
        double Q = 0.10;
        double[][] P2 = {{0.0, 0.0}, {0.0, 0.0}};

        for (int step = 0; step < 250; step++) {
            double[][] AP = matMul(A, P2);
            double[][] APAT = matMul(AP, transpose(A));
            double[][] GQGT = {
                {G[0][0] * Q * G[0][0], G[0][0] * Q * G[1][0]},
                {G[1][0] * Q * G[0][0], G[1][0] * Q * G[1][0]}
            };
            P2 = matAdd(APAT, GQGT);
        }

        System.out.println("\n2x2 covariance after 250 steps:");
        System.out.printf("[%.6f, %.6f]%n", P2[0][0], P2[0][1]);
        System.out.printf("[%.6f, %.6f]%n", P2[1][0], P2[1][1]);
    }

    static double[][] matMul(double[][] A, double[][] B) {
        int r = A.length;
        int c = B[0].length;
        int n = B.length;
        double[][] C = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                double s = 0.0;
                for (int k = 0; k < n; k++) s += A[i][k] * B[k][j];
                C[i][j] = s;
            }
        }
        return C;
    }

    static double[][] transpose(double[][] A) {
        double[][] T = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                T[j][i] = A[i][j];
        return T;
    }

    static double[][] matAdd(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }
}
