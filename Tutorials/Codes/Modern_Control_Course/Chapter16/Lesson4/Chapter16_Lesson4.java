/*
Chapter16_Lesson4.java

Construct example continuous-time SISO systems in controllable canonical form
and verify the rank of the controllability matrix.

Build:
    javac Chapter16_Lesson4.java
    java Chapter16_Lesson4
*/

import java.util.Arrays;

public class Chapter16_Lesson4 {
    static class StateSpace {
        double[][] A, B, C, D;
        StateSpace(double[][] A, double[][] B, double[][] C, double[][] D) {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;
        }
    }

    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] identity(int n) {
        double[][] I = zeros(n, n);
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int m = A[0].length;
        int c = B[0].length;
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int k = 0; k < m; k++)
                for (int j = 0; j < c; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double determinant(double[][] input) {
        int n = input.length;
        double[][] A = copy(input);
        double det = 1.0;

        for (int i = 0; i < n; i++) {
            int pivot = i;
            for (int r = i + 1; r < n; r++) {
                if (Math.abs(A[r][i]) > Math.abs(A[pivot][i])) pivot = r;
            }
            if (Math.abs(A[pivot][i]) < 1e-12) return 0.0;
            if (pivot != i) {
                double[] tmp = A[pivot];
                A[pivot] = A[i];
                A[i] = tmp;
                det = -det;
            }
            det *= A[i][i];

            for (int r = i + 1; r < n; r++) {
                double factor = A[r][i] / A[i][i];
                for (int c = i; c < n; c++) A[r][c] -= factor * A[i][c];
            }
        }
        return det;
    }

    static int rank(double[][] input) {
        double[][] A = copy(input);
        int rows = A.length;
        int cols = A[0].length;
        int r = 0;

        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(A[i][c]) > Math.abs(A[pivot][c])) pivot = i;
            }
            if (Math.abs(A[pivot][c]) < 1e-10) continue;

            double[] tmp = A[pivot];
            A[pivot] = A[r];
            A[r] = tmp;

            double piv = A[r][c];
            for (int j = c; j < cols; j++) A[r][j] /= piv;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = A[i][c];
                for (int j = c; j < cols; j++) A[i][j] -= factor * A[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] copy(double[][] M) {
        double[][] out = new double[M.length][M[0].length];
        for (int i = 0; i < M.length; i++) out[i] = Arrays.copyOf(M[i], M[i].length);
        return out;
    }

    static StateSpace ccf(double[] denDesc, double[] numDesc) {
        if (denDesc.length == 0 || Math.abs(denDesc[0]) < 1e-14) {
            throw new IllegalArgumentException("Leading denominator coefficient must be nonzero.");
        }
        int n = denDesc.length - 1;
        if (numDesc.length > n) {
            throw new IllegalArgumentException("Only strictly proper systems are handled.");
        }

        double[] den = new double[denDesc.length];
        double[] num = new double[numDesc.length];
        for (int i = 0; i < den.length; i++) den[i] = denDesc[i] / denDesc[0];
        for (int i = 0; i < num.length; i++) num[i] = numDesc[i] / denDesc[0];

        double[][] A = zeros(n, n);
        for (int i = 0; i < n - 1; i++) A[i][i + 1] = 1.0;
        for (int j = 0; j < n; j++) A[n - 1][j] = -den[n - j];

        double[][] B = zeros(n, 1);
        B[n - 1][0] = 1.0;

        double[][] C = zeros(1, n);
        int offset = n - num.length;
        for (int k = 0; k < num.length; k++) {
            int descendingIndex = offset + k;
            int ascendingIndex = n - 1 - descendingIndex;
            C[0][ascendingIndex] = num[k];
        }

        return new StateSpace(A, B, C, zeros(1, 1));
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] W = zeros(n, n);
        double[][] Ak = identity(n);

        for (int k = 0; k < n; k++) {
            double[][] col = multiply(Ak, B);
            for (int i = 0; i < n; i++) W[i][k] = col[i][0];
            Ak = multiply(A, Ak);
        }
        return W;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double x : row) System.out.printf("%12.6f ", x);
            System.out.println();
        }
    }

    static void runExample(String name, double[] den, double[] num) {
        StateSpace sys = ccf(den, num);
        double[][] W = controllabilityMatrix(sys.A, sys.B);

        System.out.println("\n" + "=".repeat(72));
        System.out.println(name);
        printMatrix("A", sys.A);
        printMatrix("B", sys.B);
        printMatrix("C", sys.C);
        printMatrix("Wc", W);
        System.out.println("rank(Wc) = " + rank(W));
        System.out.println("det(Wc)  = " + determinant(W));
    }

    public static void main(String[] args) {
        runExample("Example 1: G1(s) = 2/(s^2 + 3s + 2)", new double[]{1, 3, 2}, new double[]{2});
        runExample("Example 2: G2(s) = (s + 2)/(s^3 + 6s^2 + 11s + 6)", new double[]{1, 6, 11, 6}, new double[]{1, 2});
        runExample("Example 3: G3(s) = (0.5s^2 + 1.5s + 1)/(s^3 + 4s^2 + 5s + 2)", new double[]{1, 4, 5, 2}, new double[]{0.5, 1.5, 1.0});
    }
}
