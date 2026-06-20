// Chapter13_Lesson4.java
// Relationship of Observability to Sensor Placement
// Scratch implementation: observability matrix and numerical rank.
// Compile: javac Chapter13_Lesson4.java
// Run:     java Chapter13_Lesson4

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Chapter13_Lesson4 {
    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int p = A[0].length;
        int n = B[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < n; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] stack(List<double[][]> blocks) {
        int cols = blocks.get(0)[0].length;
        int rows = 0;
        for (double[][] block : blocks) rows += block.length;
        double[][] out = new double[rows][cols];
        int r = 0;
        for (double[][] block : blocks) {
            for (double[] row : block) {
                out[r++] = Arrays.copyOf(row, cols);
            }
        }
        return out;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        double[][] Ak = eye(n);
        List<double[][]> blocks = new ArrayList<>();
        for (int k = 0; k < n; k++) {
            blocks.add(multiply(C, Ak));
            Ak = multiply(Ak, A);
        }
        return stack(blocks);
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++) M[i] = Arrays.copyOf(input[i], cols);

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            }
            if (Math.abs(M[pivot][c]) <= tol) continue;

            double[] temp = M[r];
            M[r] = M[pivot];
            M[pivot] = temp;

            double piv = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= piv;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++) M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] cFromMask(int mask, int n) {
        int count = Integer.bitCount(mask);
        double[][] C = new double[count][n];
        int row = 0;
        for (int j = 0; j < n; j++) {
            if ((mask & (1 << j)) != 0) {
                C[row][j] = 1.0;
                row++;
            }
        }
        return C;
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0, 0.0},
            {-2.0, -0.4, 0.8, 0.0},
            {0.0, 0.0, -1.0, 1.0},
            {0.6, 0.0, -3.0, -0.5}
        };

        int n = A.length;
        System.out.println("Sensor placement by rank of O = [C; C A; ...; C A^(n-1)]");

        for (int mask = 1; mask < (1 << n); mask++) {
            if (Integer.bitCount(mask) > 2) continue;
            double[][] C = cFromMask(mask, n);
            double[][] O = observabilityMatrix(A, C);
            int r = rank(O, 1e-9);

            System.out.print("Sensors {");
            boolean first = true;
            for (int j = 0; j < n; j++) {
                if ((mask & (1 << j)) != 0) {
                    if (!first) System.out.print(",");
                    System.out.print(j + 1);
                    first = false;
                }
            }
            System.out.println("} rank = " + r);
        }
    }
}
