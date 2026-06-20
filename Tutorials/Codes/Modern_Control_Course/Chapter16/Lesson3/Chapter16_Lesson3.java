/*
Chapter16_Lesson3.java

Properties of Controllable Canonical Form (CCF) for analysis and design.

This implementation uses plain Java arrays and demonstrates:
1. companion/CCF matrix construction,
2. controllability matrix construction,
3. determinant/rank by Gaussian elimination,
4. coefficient-matching state-feedback gain.

For larger systems, consider Apache Commons Math, EJML, or ojAlgo.
*/

import java.util.Arrays;

public class Chapter16_Lesson3 {
    static double[][] zeros(int rows, int cols) {
        return new double[rows][cols];
    }

    static double[][] buildAccf(double[] a) {
        int n = a.length;
        double[][] A = zeros(n, n);
        for (int i = 0; i < n - 1; i++) A[i][i + 1] = 1.0;
        for (int j = 0; j < n; j++) A[n - 1][j] = -a[j];
        return A;
    }

    static double[] buildBccf(int n) {
        double[] B = new double[n];
        B[n - 1] = 1.0;
        return B;
    }

    static double[] matVec(double[][] A, double[] x) {
        int n = A.length;
        int m = x.length;
        double[] y = new double[n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                y[i] += A[i][j] * x[j];
        return y;
    }

    static double[][] controllabilityMatrix(double[][] A, double[] B) {
        int n = A.length;
        double[][] W = zeros(n, n);
        double[] v = B.clone();
        for (int col = 0; col < n; col++) {
            for (int row = 0; row < n; row++) W[row][col] = v[row];
            v = matVec(A, v);
        }
        return W;
    }

    static double[][] copy(double[][] M) {
        double[][] out = new double[M.length][M[0].length];
        for (int i = 0; i < M.length; i++) out[i] = M[i].clone();
        return out;
    }

    static double determinant(double[][] input) {
        double[][] M = copy(input);
        int n = M.length;
        double det = 1.0;
        int sign = 1;
        double eps = 1e-12;

        for (int k = 0; k < n; k++) {
            int pivot = k;
            for (int i = k + 1; i < n; i++)
                if (Math.abs(M[i][k]) > Math.abs(M[pivot][k])) pivot = i;

            if (Math.abs(M[pivot][k]) < eps) return 0.0;
            if (pivot != k) {
                double[] tmp = M[pivot];
                M[pivot] = M[k];
                M[k] = tmp;
                sign *= -1;
            }

            double p = M[k][k];
            det *= p;
            for (int i = k + 1; i < n; i++) {
                double factor = M[i][k] / p;
                for (int j = k; j < n; j++) M[i][j] -= factor * M[k][j];
            }
        }
        return sign * det;
    }

    static int rank(double[][] input) {
        double[][] M = copy(input);
        int rows = M.length;
        int cols = M[0].length;
        int rank = 0;
        double eps = 1e-10;

        for (int col = 0; col < cols && rank < rows; col++) {
            int pivot = rank;
            for (int i = rank + 1; i < rows; i++)
                if (Math.abs(M[i][col]) > Math.abs(M[pivot][col])) pivot = i;

            if (Math.abs(M[pivot][col]) < eps) continue;

            double[] tmp = M[pivot];
            M[pivot] = M[rank];
            M[rank] = tmp;

            double p = M[rank][col];
            for (int j = col; j < cols; j++) M[rank][j] /= p;

            for (int i = 0; i < rows; i++) {
                if (i == rank) continue;
                double factor = M[i][col];
                for (int j = col; j < cols; j++) M[i][j] -= factor * M[rank][j];
            }
            rank++;
        }
        return rank;
    }

    static double[] polePlacementGainCcf(double[] a, double[] alpha) {
        if (a.length != alpha.length) throw new IllegalArgumentException("Size mismatch.");
        double[] K = new double[a.length];
        for (int i = 0; i < a.length; i++) K[i] = alpha[i] - a[i];
        return K;
    }

    static void printMatrix(double[][] M, String name) {
        System.out.println(name + " =");
        for (double[] row : M) System.out.println(Arrays.toString(row));
    }

    public static void main(String[] args) {
        // D(s) = s^4 + 6 s^3 + 11 s^2 + 6 s + 2
        double[] a = {2.0, 6.0, 11.0, 6.0};

        double[][] A = buildAccf(a);
        double[] B = buildBccf(a.length);
        double[][] Wc = controllabilityMatrix(A, B);

        printMatrix(A, "A");
        System.out.println("B = " + Arrays.toString(B));
        printMatrix(Wc, "Wc");
        System.out.println("rank(Wc) = " + rank(Wc));
        System.out.println("det(Wc) = " + determinant(Wc));

        // D_des(s) = s^4 + 14s^3 + 71s^2 + 154s + 120
        double[] alpha = {120.0, 154.0, 71.0, 14.0};
        double[] K = polePlacementGainCcf(a, alpha);
        System.out.println("desired alpha = " + Arrays.toString(alpha));
        System.out.println("K for u = -Kx + r = " + Arrays.toString(K));
    }
}
