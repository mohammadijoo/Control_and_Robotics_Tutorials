/*
Chapter24_Lesson1.java
Existence conditions for MIMO pole assignment.

This file uses only core Java and implements the Kalman rank test and the
PBH rank test for real lambdas. For production numerical work, use EJML,
Apache Commons Math, ojAlgo, or a Java binding to LAPACK.
*/

import java.util.Arrays;

public class Chapter24_Lesson1 {
    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length;
        int p = A[0].length;
        int m = B[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < m; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) {
            I[i][i] = 1.0;
        }
        return I;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        int m = B[0].length;
        double[][] C = new double[n][n * m];
        double[][] Apow = identity(n);
        for (int block = 0; block < n; block++) {
            double[][] AB = multiply(Apow, B);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    C[i][block * m + j] = AB[i][j];
                }
            }
            Apow = multiply(A, Apow);
        }
        return C;
    }

    static int rank(double[][] input, double tol) {
        double[][] A = new double[input.length][input[0].length];
        for (int i = 0; i < input.length; i++) {
            A[i] = Arrays.copyOf(input[i], input[i].length);
        }
        int rows = A.length;
        int cols = A[0].length;
        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(A[i][c]) > Math.abs(A[pivot][c])) {
                    pivot = i;
                }
            }
            if (Math.abs(A[pivot][c]) <= tol) {
                continue;
            }
            double[] tmp = A[r];
            A[r] = A[pivot];
            A[pivot] = tmp;
            double div = A[r][c];
            for (int j = c; j < cols; j++) {
                A[r][j] /= div;
            }
            for (int i = 0; i < rows; i++) {
                if (i != r) {
                    double factor = A[i][c];
                    for (int j = c; j < cols; j++) {
                        A[i][j] -= factor * A[r][j];
                    }
                }
            }
            r++;
        }
        return r;
    }

    static double[][] pbhMatrixReal(double[][] A, double[][] B, double lambda) {
        int n = A.length;
        int m = B[0].length;
        double[][] M = new double[n][n + m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                M[i][j] = (i == j ? lambda : 0.0) - A[i][j];
            }
            for (int j = 0; j < m; j++) {
                M[i][n + j] = B[i][j];
            }
        }
        return M;
    }

    static boolean pbhAtRealLambda(double[][] A, double[][] B, double lambda) {
        return rank(pbhMatrixReal(A, B, lambda), 1e-10) == A.length;
    }

    static double[][] subtract(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }
        return C;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + ":");
        for (double[] row : M) {
            System.out.println(Arrays.toString(row));
        }
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {-1.0, -5.0, -6.0}
        };
        double[][] B = {
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 1.0}
        };

        double[][] C = controllabilityMatrix(A, B);
        printMatrix("Controllability matrix", C);
        System.out.println("Rank(C) = " + rank(C, 1e-10));
        System.out.println("Kalman controllable = " + (rank(C, 1e-10) == A.length));
        System.out.println("PBH rank condition at lambda=-2: " + pbhAtRealLambda(A, B, -2.0));

        double[][] K = {
            {8.0, 6.0, 1.0},
            {-1.0, -5.0, -3.0}
        };
        double[][] Acl = subtract(A, multiply(B, K));
        printMatrix("A - B K for a gain assigning poles -2, -3, -4", Acl);
        System.out.println("The closed-loop matrix is block triangular with eigenvalues -2, -4, -3.");

        double[][] A_bad = {
            {1.0, 0.0, 0.0},
            {0.0, -2.0, 0.0},
            {0.0, 0.0, -3.0}
        };
        double[][] B_bad = {
            {0.0},
            {1.0},
            {1.0}
        };
        System.out.println("Bad pair rank = " + rank(controllabilityMatrix(A_bad, B_bad), 1e-10));
        System.out.println("PBH at unstable lambda=1 = " + pbhAtRealLambda(A_bad, B_bad, 1.0));
    }
}
