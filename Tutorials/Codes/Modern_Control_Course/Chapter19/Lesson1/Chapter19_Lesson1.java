/*
Chapter19_Lesson1.java
Modern Control - Chapter 19, Lesson 1
Controllable/Uncontrollable Subspaces from scratch.

Compile and run:
    javac Chapter19_Lesson1.java
    java Chapter19_Lesson1

For larger numerical control projects, use EJML, Apache Commons Math, or jblas.
*/

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Chapter19_Lesson1 {
    static final double TOL = 1e-10;

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

    static double[][] hstack(List<double[][]> blocks) {
        int rows = blocks.get(0).length;
        int cols = 0;
        for (double[][] M : blocks) {
            cols += M[0].length;
        }
        double[][] H = new double[rows][cols];
        int c0 = 0;
        for (double[][] M : blocks) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < M[0].length; j++) {
                    H[i][c0 + j] = M[i][j];
                }
            }
            c0 += M[0].length;
        }
        return H;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        List<double[][]> blocks = new ArrayList<>();
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            blocks.add(multiply(Ak, B));
            Ak = multiply(Ak, A);
        }
        return hstack(blocks);
    }

    static int rank(double[][] M) {
        double[][] A = copy(M);
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
            if (Math.abs(A[pivot][c]) <= TOL) {
                continue;
            }
            double[] temp = A[r];
            A[r] = A[pivot];
            A[pivot] = temp;
            double pv = A[r][c];
            for (int j = c; j < cols; j++) {
                A[r][j] /= pv;
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

    static double[][] independentColumnBasis(double[][] M) {
        int rows = M.length;
        List<double[]> cols = new ArrayList<>();
        double[][] current = new double[rows][0];
        int currentRank = 0;
        for (int j = 0; j < M[0].length; j++) {
            double[][] candidate = appendColumn(current, column(M, j));
            int newRank = rank(candidate);
            if (newRank > currentRank) {
                cols.add(column(M, j));
                current = candidate;
                currentRank = newRank;
            }
        }
        return columnsToMatrix(cols, rows);
    }

    static double[][] completeBasis(double[][] basis, int n) {
        double[][] T = basis;
        int currentRank = rank(T);
        for (int j = 0; j < n; j++) {
            double[] e = new double[n];
            e[j] = 1.0;
            double[][] candidate = appendColumn(T, e);
            int newRank = rank(candidate);
            if (newRank > currentRank) {
                T = candidate;
                currentRank = newRank;
            }
            if (currentRank == n) {
                break;
            }
        }
        return T;
    }

    static double[][] inverse(double[][] M) {
        int n = M.length;
        double[][] A = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                A[i][j] = M[i][j];
            }
            A[i][n + i] = 1.0;
        }
        for (int c = 0; c < n; c++) {
            int pivot = c;
            for (int i = c + 1; i < n; i++) {
                if (Math.abs(A[i][c]) > Math.abs(A[pivot][c])) {
                    pivot = i;
                }
            }
            if (Math.abs(A[pivot][c]) <= TOL) {
                throw new IllegalArgumentException("Matrix is singular.");
            }
            double[] temp = A[c];
            A[c] = A[pivot];
            A[pivot] = temp;
            double pv = A[c][c];
            for (int j = 0; j < 2 * n; j++) {
                A[c][j] /= pv;
            }
            for (int i = 0; i < n; i++) {
                if (i != c) {
                    double factor = A[i][c];
                    for (int j = 0; j < 2 * n; j++) {
                        A[i][j] -= factor * A[c][j];
                    }
                }
            }
        }
        double[][] Inv = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                Inv[i][j] = A[i][n + j];
            }
        }
        return Inv;
    }

    static double[][] copy(double[][] M) {
        double[][] C = new double[M.length][M[0].length];
        for (int i = 0; i < M.length; i++) {
            C[i] = Arrays.copyOf(M[i], M[i].length);
        }
        return C;
    }

    static double[] column(double[][] M, int j) {
        double[] c = new double[M.length];
        for (int i = 0; i < M.length; i++) {
            c[i] = M[i][j];
        }
        return c;
    }

    static double[][] appendColumn(double[][] M, double[] col) {
        int rows = col.length;
        int cols = M[0].length;
        double[][] N = new double[rows][cols + 1];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                N[i][j] = M[i][j];
            }
            N[i][cols] = col[i];
        }
        return N;
    }

    static double[][] columnsToMatrix(List<double[]> cols, int rows) {
        double[][] M = new double[rows][cols.size()];
        for (int j = 0; j < cols.size(); j++) {
            for (int i = 0; i < rows; i++) {
                M[i][j] = cols.get(j)[i];
            }
        }
        return M;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println("\n" + name + " =");
        for (double[] row : M) {
            for (double v : row) {
                System.out.printf("%10.5f ", v);
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, -2.0}
        };
        double[][] B = {
            {0.0},
            {1.0},
            {0.0}
        };

        double[][] Wc = controllabilityMatrix(A, B);
        int r = rank(Wc);
        int n = A.length;
        double[][] Qc = independentColumnBasis(Wc);
        double[][] T = completeBasis(Qc, n);
        double[][] Tinv = inverse(T);
        double[][] Abar = multiply(multiply(Tinv, A), T);
        double[][] Bbar = multiply(Tinv, B);

        printMatrix("A", A);
        printMatrix("B", B);
        printMatrix("Wc = [B AB ... A^(n-1)B]", Wc);
        System.out.println("\nRank = " + r + " out of n = " + n);
        System.out.println(r == n ? "System is controllable." : "System is not controllable.");
        printMatrix("Basis for controllable subspace", Qc);
        printMatrix("T = [controllable basis, complement]", T);
        printMatrix("Abar = inv(T) A T", Abar);
        printMatrix("Bbar = inv(T) B", Bbar);
    }
}
