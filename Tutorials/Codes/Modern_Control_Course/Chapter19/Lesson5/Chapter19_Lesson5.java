// Chapter19_Lesson5.java
// Physical Interpretation of Decomposed Subsystems in Kalman Decomposition
// Self-contained Java implementation of controllability and observability ranks.

import java.util.ArrayList;
import java.util.List;

public class Chapter19_Lesson5 {
    static double[][] zeros(int r, int c) { return new double[r][c]; }
    static double[][] eye(int n) { double[][] I = zeros(n, n); for (int i = 0; i < n; i++) I[i][i] = 1.0; return I; }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, m = A[0].length, c = B[0].length;
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int k = 0; k < m; k++)
                for (int j = 0; j < c; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] hstack(List<double[][]> blocks) {
        int rows = blocks.get(0).length, cols = 0;
        for (double[][] M : blocks) cols += M[0].length;
        double[][] H = zeros(rows, cols);
        int offset = 0;
        for (double[][] M : blocks) {
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < M[0].length; j++)
                    H[i][offset + j] = M[i][j];
            offset += M[0].length;
        }
        return H;
    }

    static double[][] vstack(List<double[][]> blocks) {
        int cols = blocks.get(0)[0].length, rows = 0;
        for (double[][] M : blocks) rows += M.length;
        double[][] V = zeros(rows, cols);
        int offset = 0;
        for (double[][] M : blocks) {
            for (int i = 0; i < M.length; i++)
                for (int j = 0; j < cols; j++)
                    V[offset + i][j] = M[i][j];
            offset += M.length;
        }
        return V;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length, cols = input[0].length;
        double[][] M = zeros(rows, cols);
        for (int i = 0; i < rows; i++) System.arraycopy(input[i], 0, M[i], 0, cols);
        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++)
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            if (Math.abs(M[pivot][c]) < tol) continue;
            double[] tmp = M[r]; M[r] = M[pivot]; M[pivot] = tmp;
            double div = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= div;
            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++) M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        List<double[][]> blocks = new ArrayList<>();
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) { blocks.add(multiply(Ak, B)); Ak = multiply(Ak, A); }
        return hstack(blocks);
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        List<double[][]> blocks = new ArrayList<>();
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) { blocks.add(multiply(C, Ak)); Ak = multiply(Ak, A); }
        return vstack(blocks);
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0, 0.2, 0.0},
            {-2.0, -3.0, 0.0, 0.0, 0.0},
            {0.0, 0.3, -4.0, 0.1, 0.0},
            {0.0, 0.0, 0.0, -0.5, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.2}
        };
        double[][] B = {{0.0}, {1.0}, {1.0}, {0.0}, {0.0}};
        double[][] C = {{1.0, 0.0, 0.0, 1.0, 0.0}};

        System.out.println("rank(Wc) = " + rank(controllabilityMatrix(A, B), 1e-9) + " out of 5");
        System.out.println("rank(Wo) = " + rank(observabilityMatrix(A, C), 1e-9) + " out of 5");
        System.out.println("Reachable states: co plus c_no. Observable states: co plus no_o.");
        System.out.println("Only the co block contributes to the zero-initial transfer function.");
    }
}
