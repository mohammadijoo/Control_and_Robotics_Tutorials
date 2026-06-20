// Chapter13_Lesson2.java
// Observable states and observable subspace from scratch using basic linear algebra.
// For production-scale projects, use EJML, Apache Commons Math, or ojAlgo.

import java.util.Arrays;

public class Chapter13_Lesson2 {
    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length, n = A[0].length, p = B[0].length;
        double[][] C = new double[m][p];
        for (int i = 0; i < m; i++) {
            for (int k = 0; k < n; k++) {
                for (int j = 0; j < p; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] observabilitySignatureMatrix(double[][] A, double[][] C) {
        int n = A.length;
        int p = C.length;
        double[][] O = new double[p * n][n];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            double[][] block = multiply(C, Ak);
            for (int i = 0; i < p; i++) {
                System.arraycopy(block[i], 0, O[k * p + i], 0, n);
            }
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static double[][] rref(double[][] M, double tol) {
        int m = M.length, n = M[0].length;
        double[][] R = new double[m][n];
        for (int i = 0; i < m; i++) R[i] = Arrays.copyOf(M[i], n);
        int lead = 0;
        for (int r = 0; r < m && lead < n; r++) {
            int i = r;
            while (i < m && Math.abs(R[i][lead]) < tol) i++;
            if (i == m) { lead++; r--; continue; }
            double[] temp = R[r]; R[r] = R[i]; R[i] = temp;
            double pivot = R[r][lead];
            for (int j = 0; j < n; j++) R[r][j] /= pivot;
            for (int ii = 0; ii < m; ii++) {
                if (ii == r) continue;
                double factor = R[ii][lead];
                for (int j = 0; j < n; j++) R[ii][j] -= factor * R[r][j];
            }
            lead++;
        }
        return R;
    }

    static int rank(double[][] M, double tol) {
        double[][] R = rref(M, tol);
        int rank = 0;
        for (double[] row : R) {
            boolean nonzero = false;
            for (double v : row) if (Math.abs(v) > tol) { nonzero = true; break; }
            if (nonzero) rank++;
        }
        return rank;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + ":");
        for (double[] row : M) System.out.println(Arrays.toString(row));
        System.out.println();
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {-2.0, -3.0, 0.0},
            {0.0, 0.0, -4.0}
        };
        double[][] C = {{1.0, 0.0, 0.0}};

        double[][] O = observabilitySignatureMatrix(A, C);
        printMatrix("Observability signature matrix O", O);
        System.out.println("rank(O) = " + rank(O, 1e-10));
        System.out.println("unobservable dimension = " + (A.length - rank(O, 1e-10)));
        System.out.println("For this example, the unobservable direction is span{[0, 0, 1]^T}.");
    }
}
