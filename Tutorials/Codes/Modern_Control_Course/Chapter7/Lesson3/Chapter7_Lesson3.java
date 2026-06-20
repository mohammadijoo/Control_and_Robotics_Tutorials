/*
Chapter7_Lesson3.java
Modern Control — Chapter 7 (Solutions of LTI State Equations), Lesson 3
Solutions for Constant, Step, and Polynomial Inputs

This file is self-contained (no external linear algebra libraries), intended for
small matrices. For production use, prefer a library such as:
  - EJML (Efficient Java Matrix Library)
  - Apache Commons Math
  - ojAlgo

We implement:
  - Basic dense matrix operations
  - Matrix exponential via scaling-and-squaring + truncated Taylor series (pedagogical)
  - phi_m(Z) via truncated series

Then we compute:
  1) constant input u(t)=u0
  2) step input u(t)=u0 * 1(t-ts)
  3) polynomial input u(t)=u0 + u1 t

Compile:
  javac Chapter7_Lesson3.java
Run:
  java Chapter7_Lesson3
*/

import java.util.Arrays;

public class Chapter7_Lesson3 {

    // ----------- Basic dense matrix utilities (double[][]) -----------

    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] copy(double[][] A) {
        double[][] B = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) B[i] = Arrays.copyOf(A[i], A[i].length);
        return B;
    }

    static double[][] add(double[][] A, double[][] B) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] sub(double[][] A, double[][] B) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = s * A[i][j];
        return C;
    }

    static double[] mul(double[][] A, double[] x) {
        int n = A.length, m = A[0].length;
        if (x.length != m) throw new IllegalArgumentException("dim mismatch");
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            double sum = 0.0;
            for (int j = 0; j < m; j++) sum += A[i][j] * x[j];
            y[i] = sum;
        }
        return y;
    }

    static double[][] mul(double[][] A, double[][] B) {
        int n = A.length, p = A[0].length, m = B[0].length;
        if (B.length != p) throw new IllegalArgumentException("dim mismatch");
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < p; k++) {
                double aik = A[i][k];
                for (int j = 0; j < m; j++) C[i][j] += aik * B[k][j];
            }
        }
        return C;
    }

    static double norm1(double[][] A) {
        // Matrix 1-norm: max column sum
        int n = A.length, m = A[0].length;
        double max = 0.0;
        for (int j = 0; j < m; j++) {
            double col = 0.0;
            for (int i = 0; i < n; i++) col += Math.abs(A[i][j]);
            max = Math.max(max, col);
        }
        return max;
    }

    static double factorial(int n) {
        double f = 1.0;
        for (int k = 2; k <= n; k++) f *= (double) k;
        return f;
    }

    // ----------- Matrix exponential (scaling/squaring + Taylor) -----------

    static double[][] expm(double[][] A) {
        // Pedagogical: scale A by 2^{-s} to make norm small, then use Taylor, then square.
        int n = A.length;
        double aNorm = norm1(A);
        int s = 0;
        if (aNorm > 0.5) {
            s = (int) Math.ceil(Math.log(aNorm / 0.5) / Math.log(2.0));
        }

        double[][] As = scale(A, 1.0 / Math.pow(2.0, s));

        // Taylor series exp(As) ≈ Σ_{k=0}^{K} As^k/k!
        int K = 30;
        double[][] X = eye(n);
        double[][] term = eye(n);
        for (int k = 1; k <= K; k++) {
            term = mul(term, As);
            X = add(X, scale(term, 1.0 / factorial(k)));
        }

        // Undo scaling by squaring s times
        for (int i = 0; i < s; i++) X = mul(X, X);
        return X;
    }

    // ----------- phi-functions via series -----------

    static double[][] phiSeries(double[][] Z, int m, int terms) {
        int n = Z.length;
        double[][] I = eye(n);
        double[][] S = scale(I, 1.0 / factorial(m)); // j=0
        double[][] Zpow = eye(n);

        for (int j = 1; j < terms; j++) {
            Zpow = mul(Zpow, Z);
            S = add(S, scale(Zpow, 1.0 / factorial(j + m)));
        }
        return S;
    }

    // ----------- Closed-form responses -----------

    static double[] xConstant(double[][] A, double[][] B, double[] x0, double[] u0, double t) {
        double[][] At = scale(A, t);
        double[][] E = expm(At);
        double[][] Phi1 = phiSeries(At, 1, 40);

        double[] Bu0 = mul(B, u0);
        double[] forced = mul(scale(Phi1, t), Bu0);

        double[] x = mul(E, x0);
        for (int i = 0; i < x.length; i++) x[i] += forced[i];
        return x;
    }

    static double[] xStep(double[][] A, double[][] B, double[] x0, double[] u0, double t, double ts) {
        if (t < ts) {
            return mul(expm(scale(A, t)), x0);
        } else {
            double[][] E = expm(scale(A, t));
            double dt = t - ts;
            double[][] Phi1 = phiSeries(scale(A, dt), 1, 40);

            double[] Bu0 = mul(B, u0);
            double[] forced = mul(scale(Phi1, dt), Bu0);

            double[] x = mul(E, x0);
            for (int i = 0; i < x.length; i++) x[i] += forced[i];
            return x;
        }
    }

    static double[] xPoly(double[][] A, double[][] B, double[] x0, double[][] coeffs, double t) {
        // u(t) = sum_{k=0}^p u_k t^k
        // x(t) = e^{At} x0 + sum_{k=0}^p t^{k+1} k! * phi_{k+1}(At) B u_k
        double[][] At = scale(A, t);
        double[][] E = expm(At);
        double[] x = mul(E, x0);

        for (int k = 0; k < coeffs.length; k++) {
            double[][] Phik1 = phiSeries(At, k + 1, 45);
            double[] Buk = mul(B, coeffs[k]);

            double scaleFac = Math.pow(t, k + 1) * factorial(k);
            double[] add = mul(scale(Phik1, scaleFac), Buk);
            for (int i = 0; i < x.length; i++) x[i] += add[i];
        }
        return x;
    }

    static String fmt(double[] x) {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < x.length; i++) {
            sb.append(String.format("% .6f", x[i]));
            if (i + 1 < x.length) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }

    public static void main(String[] args) {
        // Example: 2-state, 1-input
        double[][] A = new double[][] {
                {0.0, 1.0},
                {-2.0, -3.0}
        };
        double[][] B = new double[][] {
                {0.0},
                {1.0}
        };
        double[] x0 = new double[] {1.0, 0.0};
        double[] u0 = new double[] {2.0};
        double[] u1 = new double[] {0.5};

        double[][] coeffs = new double[][] {u0, u1};

        double ts = 1.5;

        double[] times = new double[] {0.0, 1.0, 2.0, 5.0};
        for (double t : times) {
            double[] xc = xConstant(A, B, x0, u0, t);
            double[] xs = xStep(A, B, x0, u0, t, ts);
            double[] xp = xPoly(A, B, x0, coeffs, t);

            System.out.println("t = " + t);
            System.out.println("  x_const = " + fmt(xc));
            System.out.println("  x_step  = " + fmt(xs));
            System.out.println("  x_poly  = " + fmt(xp));
            System.out.println();
        }
    }
}
