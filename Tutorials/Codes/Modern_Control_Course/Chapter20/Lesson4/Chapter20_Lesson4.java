// Chapter20_Lesson4.java
// Library-free educational implementation of controllability and observability ranks.
// For production Java control computation, use EJML, ojAlgo, or Apache Commons Math.

import java.util.Arrays;

public class Chapter20_Lesson4 {
    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length, n = B[0].length, p = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < n; j++) C[i][j] += A[i][k] * B[k][j];
            }
        }
        return C;
    }

    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length;
        double[][] R = new double[n][n * m];
        double[][] Apow = eye(n);
        for (int k = 0; k < n; k++) {
            double[][] block = multiply(Apow, B);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) R[i][k * m + j] = block[i][j];
            }
            Apow = multiply(Apow, A);
        }
        return R;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length, p = C.length;
        double[][] O = new double[n * p][n];
        double[][] Apow = eye(n);
        for (int k = 0; k < n; k++) {
            double[][] block = multiply(C, Apow);
            for (int i = 0; i < p; i++) {
                for (int j = 0; j < n; j++) O[k * p + i][j] = block[i][j];
            }
            Apow = multiply(Apow, A);
        }
        return O;
    }

    static int rankRref(double[][] M, double tol) {
        int rows = M.length, cols = M[0].length;
        double[][] A = new double[rows][cols];
        for (int i = 0; i < rows; i++) A[i] = Arrays.copyOf(M[i], cols);
        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(A[i][c]) > Math.abs(A[pivot][c])) pivot = i;
            }
            if (Math.abs(A[pivot][c]) <= tol) continue;
            double[] tmp = A[r]; A[r] = A[pivot]; A[pivot] = tmp;
            double piv = A[r][c];
            for (int j = c; j < cols; j++) A[r][j] /= piv;
            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = A[i][c];
                for (int j = c; j < cols; j++) A[i][j] -= factor * A[r][j];
            }
            r++;
        }
        return r;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) System.out.println(Arrays.toString(row));
        System.out.println();
    }

    public static void main(String[] args) {
        double[][] A = {
            {-1.0, 0.0, 0.0},
            { 0.0,-2.0, 0.0},
            { 0.0, 0.0,-3.0}
        };
        double[][] B = {{1.0}, {0.0}, {1.0}};
        double[][] C = {{1.0, 0.0, 0.0}};

        double[][] R = controllabilityMatrix(A, B);
        double[][] O = observabilityMatrix(A, C);
        printMatrix("Controllability matrix", R);
        printMatrix("Observability matrix", O);

        int n = A.length;
        int rankR = rankRref(R, 1e-10);
        int rankO = rankRref(O, 1e-10);
        System.out.println("rank(R) = " + rankR + " of " + n);
        System.out.println("rank(O) = " + rankO + " of " + n);
        System.out.println("Conclusion: this realization is not minimal.");
        System.out.println("For this diagonal example, the input-output map is G(s)=1/(s+1), so a minimal realization is A=[-1], B=[1], C=[1], D=[0].");
    }
}
