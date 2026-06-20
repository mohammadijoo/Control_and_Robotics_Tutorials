// Chapter18_Lesson2.java
// From-scratch rank/nullity diagnostics for Jordan block sizes.
// This is educational code, not a numerically robust Jordan-form algorithm.

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Chapter18_Lesson2 {
    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] subtractLambdaI(double[][] A, double lambda) {
        int n = A.length;
        double[][] B = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, B[i], 0, n);
            B[i][i] -= lambda;
        }
        return B;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length;
        int m = B[0].length;
        int p = B.length;
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

    static int rankGaussian(double[][] input, double tolerance) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] A = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            System.arraycopy(input[i], 0, A[i], 0, cols);
        }

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(A[i][c]) > Math.abs(A[pivot][c])) {
                    pivot = i;
                }
            }

            if (Math.abs(A[pivot][c]) <= tolerance) continue;

            double[] temp = A[r];
            A[r] = A[pivot];
            A[pivot] = temp;

            double divisor = A[r][c];
            for (int j = c; j < cols; j++) A[r][j] /= divisor;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = A[i][c];
                for (int j = c; j < cols; j++) {
                    A[i][j] -= factor * A[r][j];
                }
            }
            r++;
        }
        return r;
    }

    static List<Integer> nullitySequence(double[][] A, double lambda, int maxPower) {
        int n = A.length;
        double[][] N = subtractLambdaI(A, lambda);
        double[][] Nk = identity(n);

        List<Integer> nullities = new ArrayList<>();
        nullities.add(0);

        for (int k = 1; k <= maxPower; k++) {
            Nk = multiply(Nk, N);
            int rank = rankGaussian(Nk, 1e-10);
            nullities.add(n - rank);
        }
        return nullities;
    }

    static List<Integer> blockSizesFromNullities(List<Integer> nullities, int algebraicMultiplicity) {
        List<Integer> trimmed = new ArrayList<>();
        trimmed.add(nullities.get(0));

        for (int i = 1; i < nullities.size(); i++) {
            trimmed.add(nullities.get(i));
            if (nullities.get(i) == algebraicMultiplicity) break;
        }

        List<Integer> blocksAtLeast = new ArrayList<>();
        for (int k = 1; k < trimmed.size(); k++) {
            blocksAtLeast.add(trimmed.get(k) - trimmed.get(k - 1));
        }
        blocksAtLeast.add(0);

        List<Integer> sizes = new ArrayList<>();
        for (int k = 1; k < blocksAtLeast.size(); k++) {
            int exactCount = blocksAtLeast.get(k - 1) - blocksAtLeast.get(k);
            for (int c = 0; c < exactCount; c++) {
                sizes.add(k);
            }
        }

        sizes.sort(Collections.reverseOrder());
        return sizes;
    }

    public static void main(String[] args) {
        double[][] J = {
            { 2, 1, 0, 0,  0, 0},
            { 0, 2, 1, 0,  0, 0},
            { 0, 0, 2, 0,  0, 0},
            { 0, 0, 0, 2,  0, 0},
            { 0, 0, 0, 0, -1, 1},
            { 0, 0, 0, 0,  0,-1}
        };

        double[] eigenvalues = {2.0, -1.0};
        int[] algebraicMultiplicities = {4, 2};

        for (int i = 0; i < eigenvalues.length; i++) {
            List<Integer> nullities = nullitySequence(J, eigenvalues[i], 6);
            List<Integer> sizes = blockSizesFromNullities(nullities, algebraicMultiplicities[i]);

            System.out.println("lambda = " + eigenvalues[i]);
            System.out.println("nullities n_k = " + nullities);
            System.out.println("Jordan block sizes = " + sizes);
            System.out.println();
        }

        System.out.println("Expected: lambda 2 has block sizes [3, 1]; lambda -1 has [2].");
    }
}
