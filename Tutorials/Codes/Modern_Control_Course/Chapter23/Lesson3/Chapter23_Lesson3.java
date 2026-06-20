// Chapter23_Lesson3.java
// Ackermann's formula for SISO pole placement using basic matrix operations.
// Compile: javac Chapter23_Lesson3.java
// Run:     java Chapter23_Lesson3

import java.util.Arrays;

public class Chapter23_Lesson3 {
    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] eye(int n) {
        double[][] I = zeros(n, n);
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] add(double[][] A, double[][] B) {
        int r = A.length, c = A[0].length;
        double[][] R = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                R[i][j] = A[i][j] + B[i][j];
        return R;
    }

    static double[][] scale(double a, double[][] A) {
        int r = A.length, c = A[0].length;
        double[][] R = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                R[i][j] = a * A[i][j];
        return R;
    }

    static double[][] mul(double[][] A, double[][] B) {
        int r = A.length, m = A[0].length, c = B[0].length;
        double[][] R = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int k = 0; k < m; k++)
                for (int j = 0; j < c; j++)
                    R[i][j] += A[i][k] * B[k][j];
        return R;
    }

    static double[][] mpow(double[][] A, int p) {
        int n = A.length;
        double[][] R = eye(n);
        double[][] X = A;
        while (p > 0) {
            if ((p & 1) == 1) R = mul(R, X);
            X = mul(X, X);
            p >>= 1;
        }
        return R;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = zeros(n, 2 * n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
            aug[i][n + i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            if (Math.abs(aug[pivot][col]) < 1e-12)
                throw new RuntimeException("Matrix is singular or ill-conditioned.");
            double[] temp = aug[pivot]; aug[pivot] = aug[col]; aug[col] = temp;
            double div = aug[col][col];
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= div;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = 0; j < 2 * n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] inv = zeros(n, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                inv[i][j] = aug[i][n + j];
        return inv;
    }

    static double[][] controllability(double[][] A, double[][] B) {
        int n = A.length;
        double[][] C = zeros(n, n);
        double[][] block = B;
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) C[i][k] = block[i][0];
            block = mul(A, block);
        }
        return C;
    }

    static double[][] matrixPolynomial(double[][] A, double[] coeff) {
        int n = A.length;
        double[][] R = mpow(A, n);
        for (int i = 1; i <= n; i++) {
            int power = n - i;
            double[][] term = (power == 0) ? eye(n) : mpow(A, power);
            R = add(R, scale(coeff[i], term));
        }
        return R;
    }

    static double[] ackermannGain(double[][] A, double[][] B, double[] desiredPoly) {
        int n = A.length;
        double[][] C = controllability(A, B);
        double[][] Cinv = inverse(C);
        double[][] phiA = matrixPolynomial(A, desiredPoly);
        double[][] eT = zeros(1, n);
        eT[0][n - 1] = 1.0;
        double[][] K = mul(mul(eT, Cinv), phiA);
        return K[0];
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-2.0, -3.0}};
        double[][] B = {{0.0}, {1.0}};

        // Desired poles -2 +/- 2i give p_d(s) = s^2 + 4s + 8.
        double[] desiredPoly = {1.0, 4.0, 8.0};
        double[] K = ackermannGain(A, B, desiredPoly);
        System.out.println("K = " + Arrays.toString(K));
        System.out.println("Expected for this example: K = [6.0, 1.0]");
    }
}
