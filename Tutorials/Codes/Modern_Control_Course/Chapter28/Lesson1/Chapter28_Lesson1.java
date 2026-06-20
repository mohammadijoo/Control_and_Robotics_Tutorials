/*
Chapter28_Lesson1.java

Self-contained Java implementation for small quadratic-form calculations.
For larger control projects, consider EJML, Apache Commons Math, or JBLAS.
*/

import java.util.Arrays;

public class Chapter28_Lesson1 {
    static double dot(double[] a, double[] b) {
        double s = 0.0;
        for (int i = 0; i < a.length; i++) s += a[i] * b[i];
        return s;
    }

    static double[] matVec(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < x.length; j++) y[i] += A[i][j] * x[j];
        }
        return y;
    }

    static double quadraticForm(double[] z, double[][] M) {
        return dot(z, matVec(M, z));
    }

    static double stageCost(double[] x, double[] u, double[][] Q, double[][] R, double[][] N) {
        double value = quadraticForm(x, Q) + quadraticForm(u, R);

        // cross term: 2*x^T*N*u
        double cross = 0.0;
        for (int i = 0; i < x.length; i++) {
            for (int j = 0; j < u.length; j++) {
                cross += x[i] * N[i][j] * u[j];
            }
        }
        return value + 2.0 * cross;
    }

    static double[][] schurComplementForScalarR(double[][] Q, double r, double[][] N) {
        int n = Q.length;
        double[][] S = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                S[i][j] = Q[i][j] - N[i][0] * N[j][0] / r;
            }
        }
        return S;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name);
        for (double[] row : A) System.out.println(Arrays.toString(row));
    }

    public static void main(String[] args) {
        double[][] Q = {
            {10.0, 0.0},
            {0.0, 1.0}
        };
        double[][] R = {{0.25}};
        double[][] N = {
            {0.0},
            {0.15}
        };

        double[] x = {0.7, -0.2};
        double[] u = {0.4};

        System.out.println("x^T Q x = " + quadraticForm(x, Q));
        System.out.println("stage cost = " + stageCost(x, u, Q, R, N));

        double[][] S = schurComplementForScalarR(Q, R[0][0], N);
        printMatrix("Q - N R^{-1} N^T =", S);
    }
}
