// Chapter20_Lesson3.java
// Internal vs external equivalence of continuous-time LTI systems.
// No external library is required; this file implements small matrix utilities from scratch.

import java.util.Arrays;

public class Chapter20_Lesson3 {
    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] sub(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    static double[][] scale(double a, double[][] A) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) C[i][j] = a * A[i][j];
        return C;
    }

    static double[][] mul(double[][] A, double[][] B) {
        int m = A.length, n = B[0].length, p = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int k = 0; k < p; k++)
                for (int j = 0; j < n; j++) C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] hcat(double[][][] blocks) {
        int rows = blocks[0].length;
        int cols = 0;
        for (double[][] B : blocks) cols += B[0].length;
        double[][] out = new double[rows][cols];
        int offset = 0;
        for (double[][] B : blocks) {
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < B[0].length; j++) out[i][offset + j] = B[i][j];
            offset += B[0].length;
        }
        return out;
    }

    static double[][] vcat(double[][][] blocks) {
        int cols = blocks[0][0].length;
        int rows = 0;
        for (double[][] B : blocks) rows += B.length;
        double[][] out = new double[rows][cols];
        int offset = 0;
        for (double[][] B : blocks) {
            for (int i = 0; i < B.length; i++)
                for (int j = 0; j < cols; j++) out[offset + i][j] = B[i][j];
            offset += B.length;
        }
        return out;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
            aug[i][n + i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            double[] tmp = aug[col]; aug[col] = aug[pivot]; aug[pivot] = tmp;
            double div = aug[col][col];
            if (Math.abs(div) < 1e-12) throw new RuntimeException("singular matrix");
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= div;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = 0; j < 2 * n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) inv[i][j] = aug[i][n + j];
        return inv;
    }

    static int rank(double[][] A) {
        double[][] M = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) M[i] = Arrays.copyOf(A[i], A[i].length);
        int rows = M.length, cols = M[0].length, r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++)
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            if (Math.abs(M[pivot][c]) < 1e-10) continue;
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

    static double norm(double[][] A) {
        double s = 0.0;
        for (double[] row : A) for (double v : row) s += v * v;
        return Math.sqrt(s);
    }

    static double[][] ctrb(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = mul(Ak, B);
            Ak = mul(A, Ak);
        }
        return hcat(blocks);
    }

    static double[][] obsv(double[][] A, double[][] C) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = mul(C, Ak);
            Ak = mul(Ak, A);
        }
        return vcat(blocks);
    }

    static double transfer(double[][] A, double[][] B, double[][] C, double[][] D, double s) {
        double[][] resolvent = inverse(sub(scale(s, eye(A.length)), A));
        return add(mul(mul(C, resolvent), B), D)[0][0];
    }

    static void report(String name, double[][] A, double[][] B, double[][] C, double[][] D) {
        int n = A.length;
        System.out.println("\n" + name);
        System.out.println("rank controllability = " + rank(ctrb(A, B)) + " of " + n);
        System.out.println("rank observability   = " + rank(obsv(A, C)) + " of " + n);
        System.out.println("minimal? " + (rank(ctrb(A, B)) == n && rank(obsv(A, C)) == n));
    }

    public static void main(String[] args) {
        double[][] A1 = {{-1.0, 0.0}, {0.0, -2.0}};
        double[][] B1 = {{1.0}, {1.0}};
        double[][] C1 = {{1.0, 1.0}};
        double[][] D1 = {{0.0}};

        double[][] T = {{1.0, 2.0}, {0.5, 1.5}};
        double[][] Ti = inverse(T);
        double[][] A2 = mul(mul(T, A1), Ti);
        double[][] B2 = mul(T, B1);
        double[][] C2 = mul(C1, Ti);
        double[][] D2 = D1;

        double[][] A3 = {{-1.0, 0.0, 0.0}, {0.0, -2.0, 0.0}, {0.0, 0.0, 5.0}};
        double[][] B3 = {{1.0}, {1.0}, {0.0}};
        double[][] C3 = {{1.0, 1.0, 0.0}};
        double[][] D3 = D1;

        report("Sigma_1 minimal", A1, B1, C1, D1);
        report("Sigma_2 internally equivalent to Sigma_1", A2, B2, C2, D2);
        report("Sigma_3 externally equivalent but nonminimal", A3, B3, C3, D3);

        System.out.println("\nInternal-equivalence residuals for Sigma_1 and Sigma_2:");
        System.out.println("A residual = " + norm(sub(A2, mul(mul(T, A1), Ti))));
        System.out.println("B residual = " + norm(sub(B2, mul(T, B1))));
        System.out.println("C residual = " + norm(sub(C2, mul(C1, Ti))));
        System.out.println("D residual = " + norm(sub(D2, D1)));

        System.out.println("\nTransfer-function samples G_i(s):");
        for (double s : new double[]{0.1, 1.0, 3.0, 10.0}) {
            System.out.printf("s=%4.1f  G1=% .8f  G2=% .8f  G3=% .8f%n",
                    s,
                    transfer(A1, B1, C1, D1, s),
                    transfer(A2, B2, C2, D2, s),
                    transfer(A3, B3, C3, D3, s));
        }
    }
}
