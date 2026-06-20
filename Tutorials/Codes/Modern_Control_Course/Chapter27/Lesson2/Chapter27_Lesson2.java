// Chapter27_Lesson2.java
// Feedforward gain design with small dense-matrix utilities from scratch.

import java.util.Arrays;

public class Chapter27_Lesson2 {
    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length, q = B.length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                for (int k = 0; k < q; k++) C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] add(double[][] A, double[][] B, double alpha) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++) C[i][j] = A[i][j] + alpha * B[i][j];
        return C;
    }

    static double[][] solve(double[][] A, double[][] B) {
        int n = A.length, p = B[0].length;
        double[][] aug = new double[n][n + p];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, aug[i], 0, n);
            System.arraycopy(B[i], 0, aug[i], n, p);
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            if (Math.abs(aug[pivot][col]) < 1e-12) throw new RuntimeException("Singular matrix.");
            double[] tmp = aug[col]; aug[col] = aug[pivot]; aug[pivot] = tmp;
            double scale = aug[col][col];
            for (int j = col; j < n + p; j++) aug[col][j] /= scale;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = col; j < n + p; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] X = new double[n][p];
        for (int i = 0; i < n; i++) System.arraycopy(aug[i], n, X[i], 0, p);
        return X;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return solve(A, I);
    }

    static double[][] feedforwardFromDCGain(double[][] A, double[][] B, double[][] C,
                                            double[][] D, double[][] K) {
        double[][] Acl = add(A, multiply(B, K), -1.0);
        double[][] Ccl = add(C, multiply(D, K), -1.0);
        double[][] AclInvB = solve(Acl, B);
        double[][] G0 = add(D, multiply(Ccl, AclInvB), -1.0);
        return inverse(G0);
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name);
        for (double[] row : A) System.out.println(Arrays.toString(row));
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {0.0, 0.0}};
        double[][] B = {{0.0}, {1.0}};
        double[][] C = {{1.0, 0.0}};
        double[][] D = {{0.0}};
        double[][] K = {{4.0, 4.0}};

        double[][] N = feedforwardFromDCGain(A, B, C, D, K);
        printMatrix("N from DC gain:", N);

        double dt = 0.001, tf = 6.0;
        double[][] x = {{0.0}, {0.0}};
        double[][] r = {{1.0}};
        int steps = (int)(tf / dt);
        for (int k = 0; k < steps; k++) {
            double[][] u = add(multiply(N, r), multiply(K, x), -1.0);
            double[][] dx = add(multiply(A, x), multiply(B, u), 1.0);
            x = add(x, dx, dt);
        }
        double[][] uFinal = add(multiply(N, r), multiply(K, x), -1.0);
        double[][] yFinal = add(multiply(C, x), multiply(D, uFinal), 1.0);
        printMatrix("Final output y(tf):", yFinal);
    }
}
