/*
Chapter16_Lesson5.java
Scratch Java implementation for controllable canonical form diagnostics.

Production Java libraries for control/numerics:
  - EJML, Apache Commons Math, ojAlgo
*/

import java.util.Arrays;

public class Chapter16_Lesson5 {
    static double[][] zeros(int r, int c) {
        return new double[r][c];
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

    static double[][] companionCCF(double[] aAscending) {
        int n = aAscending.length;
        double[][] A = zeros(n, n);
        for (int i = 0; i < n - 1; i++) A[i][i + 1] = 1.0;
        for (int j = 0; j < n; j++) A[n - 1][j] = -aAscending[j];
        return A;
    }

    static double[][] inputB(int n) {
        double[][] B = zeros(n, 1);
        B[n - 1][0] = 1.0;
        return B;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Q = zeros(n, n);
        double[][] AkB = B;
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) Q[i][k] = AkB[i][0];
            AkB = multiply(A, AkB);
        }
        return Q;
    }

    static int rankGaussian(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++) M[i] = Arrays.copyOf(input[i], cols);

        int rank = 0;
        for (int col = 0; col < cols && rank < rows; col++) {
            int pivot = rank;
            for (int i = rank + 1; i < rows; i++)
                if (Math.abs(M[i][col]) > Math.abs(M[pivot][col])) pivot = i;
            if (Math.abs(M[pivot][col]) <= tol) continue;

            double[] temp = M[pivot];
            M[pivot] = M[rank];
            M[rank] = temp;

            double div = M[rank][col];
            for (int j = col; j < cols; j++) M[rank][j] /= div;

            for (int i = 0; i < rows; i++) {
                if (i == rank) continue;
                double factor = M[i][col];
                for (int j = col; j < cols; j++) M[i][j] -= factor * M[rank][j];
            }
            rank++;
        }
        return rank;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double v : row) System.out.printf("%12.6f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[] a = {6.0, 11.0, 6.0}; // s^3 + 6s^2 + 11s + 6
        double[][] A = companionCCF(a);
        double[][] B = inputB(a.length);
        double[][] Q = controllabilityMatrix(A, B);

        printMatrix("A_c", A);
        printMatrix("B_c", B);
        printMatrix("Q_c", Q);
        System.out.println("rank(Q_c) = " + rankGaussian(Q, 1e-10));

        // Desired poles -2, -3, -4 -> s^3 + 9s^2 + 26s + 24
        double[] alphaAscending = {24.0, 26.0, 9.0};
        double[] K = new double[a.length];
        for (int i = 0; i < a.length; i++) K[i] = alphaAscending[i] - a[i];
        System.out.println("K = " + Arrays.toString(K));
    }
}
