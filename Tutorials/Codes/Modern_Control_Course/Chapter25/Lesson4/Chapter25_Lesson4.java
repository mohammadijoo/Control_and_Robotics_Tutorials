/*
Chapter25_Lesson4.java
Impact of Model Uncertainty on State-Feedback Designs

Compile and run:
    javac Chapter25_Lesson4.java
    java Chapter25_Lesson4

This file implements the 2x2 calculations from scratch.
*/

import java.util.Random;

public class Chapter25_Lesson4 {
    static double[][] matMul(double[][] A, double[][] B) {
        double[][] C = new double[A.length][B[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < B[0].length; j++)
                for (int k = 0; k < B.length; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] matAdd(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scalarMul(double a, double[][] A) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                C[i][j] = a * A[i][j];
        return C;
    }

    static double[][] inverse2(double[][] M) {
        double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        if (Math.abs(det) < 1e-12) throw new RuntimeException("Singular 2x2 matrix.");
        return new double[][]{{ M[1][1]/det, -M[0][1]/det }, { -M[1][0]/det, M[0][0]/det }};
    }

    static double[] rowTimesMatrix(double[] r, double[][] M) {
        return new double[]{r[0]*M[0][0] + r[1]*M[1][0], r[0]*M[0][1] + r[1]*M[1][1]};
    }

    static double[] rowTimesMatrix2(double[] r, double[][] M1, double[][] M2) {
        return rowTimesMatrix(rowTimesMatrix(r, M1), M2);
    }

    static double[] ackermann2(double[][] A, double[][] Bcol, double p1, double p2) {
        double[][] AB = matMul(A, Bcol);
        double[][] C = {{Bcol[0][0], AB[0][0]}, {Bcol[1][0], AB[1][0]}};
        double[][] Cinv = inverse2(C);

        double a1 = -(p1 + p2);
        double a0 = p1 * p2;
        double[][] A2 = matMul(A, A);
        double[][] phiA = matAdd(matAdd(A2, scalarMul(a1, A)), scalarMul(a0, new double[][]{{1,0},{0,1}}));
        return rowTimesMatrix2(new double[]{0.0, 1.0}, Cinv, phiA);
    }

    static double[] eigRealParts2(double[][] M) {
        double tr = M[0][0] + M[1][1];
        double det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
        double disc = tr*tr - 4.0*det;
        if (disc >= 0.0) {
            double root = Math.sqrt(disc);
            return new double[]{(tr + root)/2.0, (tr - root)/2.0};
        }
        return new double[]{tr/2.0, tr/2.0};
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-2.0, -0.5}};
        double[][] B = {{0.0}, {1.0}};
        double[] K = ackermann2(A, B, -2.0, -3.0);
        System.out.printf("K = [%.6f %.6f]%n", K[0], K[1]);

        Random rng = new Random(25);
        int N = 5000;
        int unstable = 0;
        double worst = -1e9;

        for (int s = 0; s < N; s++) {
            double[][] dA = new double[2][2];
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    dA[i][j] = 0.05 * rng.nextGaussian();

            double rho = -0.25 + 0.5 * rng.nextDouble();
            double[][] Atrue = matAdd(A, dA);
            double[][] BK = {{B[0][0]*(1+rho)*K[0], B[0][0]*(1+rho)*K[1]},
                             {B[1][0]*(1+rho)*K[0], B[1][0]*(1+rho)*K[1]}};
            double[][] Acl = matAdd(Atrue, scalarMul(-1.0, BK));
            double[] rp = eigRealParts2(Acl);
            double maxReal = Math.max(rp[0], rp[1]);
            worst = Math.max(worst, maxReal);
            if (maxReal >= 0.0) unstable++;
        }

        System.out.println("Unstable samples = " + unstable + " out of " + N);
        System.out.println("Worst observed max real part = " + worst);
    }
}
