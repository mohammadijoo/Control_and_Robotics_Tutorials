/*
Chapter26_Lesson3.java
SISO augmented-state pole placement with integral action.

This file uses only small from-scratch matrix utilities for teaching purposes.
Compile and run:
  javac Chapter26_Lesson3.java
  java Chapter26_Lesson3
*/

import java.util.Arrays;

public class Chapter26_Lesson3 {
    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, c = B[0].length, inner = B.length;
        double[][] M = new double[r][c];
        for (int i = 0; i &lt; r; i++) {
            for (int j = 0; j &lt; c; j++) {
                for (int k = 0; k &lt; inner; k++) M[i][j] += A[i][k] * B[k][j];
            }
        }
        return M;
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] M = new double[A.length][A[0].length];
        for (int i = 0; i &lt; A.length; i++)
            for (int j = 0; j &lt; A[0].length; j++) M[i][j] = A[i][j] + B[i][j];
        return M;
    }

    static double[][] scale(double s, double[][] A) {
        double[][] M = new double[A.length][A[0].length];
        for (int i = 0; i &lt; A.length; i++)
            for (int j = 0; j &lt; A[0].length; j++) M[i][j] = s * A[i][j];
        return M;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i &lt; n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i &lt; n; i++) {
            for (int j = 0; j &lt; n; j++) aug[i][j] = A[i][j];
            aug[i][n + i] = 1.0;
        }
        for (int col = 0; col &lt; n; col++) {
            int pivot = col;
            for (int r = col + 1; r &lt; n; r++)
                if (Math.abs(aug[r][col]) &gt; Math.abs(aug[pivot][col])) pivot = r;
            double[] tmp = aug[col]; aug[col] = aug[pivot]; aug[pivot] = tmp;
            double div = aug[col][col];
            if (Math.abs(div) &lt; 1e-12) throw new IllegalArgumentException("Singular matrix");
            for (int j = 0; j &lt; 2 * n; j++) aug[col][j] /= div;
            for (int r = 0; r &lt; n; r++) if (r != col) {
                double factor = aug[r][col];
                for (int j = 0; j &lt; 2 * n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i &lt; n; i++)
            for (int j = 0; j &lt; n; j++) inv[i][j] = aug[i][n + j];
        return inv;
    }

    static double[][] hstack(double[][][] blocks) {
        int rows = blocks[0].length;
        int cols = 0;
        for (double[][] block : blocks) cols += block[0].length;
        double[][] H = new double[rows][cols];
        int offset = 0;
        for (double[][] block : blocks) {
            for (int i = 0; i &lt; rows; i++)
                for (int j = 0; j &lt; block[0].length; j++) H[i][offset + j] = block[i][j];
            offset += block[0].length;
        }
        return H;
    }

    static double[] polynomialFromRoots(double[] roots) {
        double[] coeff = {1.0};
        for (double r : roots) {
            double[] next = new double[coeff.length + 1];
            for (int i = 0; i &lt; coeff.length; i++) {
                next[i] += coeff[i];
                next[i + 1] += -r * coeff[i];
            }
            coeff = next;
        }
        return coeff;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = identity(n);
        for (int k = 0; k &lt; n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(Ak, A);
        }
        return hstack(blocks);
    }

    static double[][] ackermannSISO(double[][] A, double[][] B, double[] desiredPoles) {
        int n = A.length;
        double[][] CoInv = inverse(controllabilityMatrix(A, B));
        double[] coeff = polynomialFromRoots(desiredPoles);
        double[][][] powers = new double[n + 1][][];
        powers[0] = identity(n);
        for (int k = 1; k &lt;= n; k++) powers[k] = multiply(powers[k - 1], A);

        double[][] phiA = new double[n][n];
        for (int power = n; power &gt;= 0; power--) {
            phiA = add(phiA, scale(coeff[n - power], powers[power]));
        }
        double[][] en = new double[1][n];
        en[0][n - 1] = 1.0;
        return multiply(multiply(en, CoInv), phiA);
    }

    public static void main(String[] args) {
        double[][] Aaug = {
            {0.0, 1.0, 0.0},
            {-2.0, -0.6, 0.0},
            {-1.0, 0.0, 0.0}
        };
        double[][] Baug = {{0.0}, {1.0}, {0.0}};
        double[] poles = {-2.0, -2.5, -3.0};
        double[][] K = ackermannSISO(Aaug, Baug, poles);
        System.out.println("K_aug = " + Arrays.deepToString(K));
        System.out.println("Kx = [" + K[0][0] + ", " + K[0][1] + "]");
        System.out.println("Ki = " + K[0][2]);
    }
}
