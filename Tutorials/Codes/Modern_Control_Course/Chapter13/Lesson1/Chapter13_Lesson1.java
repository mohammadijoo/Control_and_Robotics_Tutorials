// Chapter13_Lesson1.java
// From-scratch observability matrix and rank test for small LTI systems.
// Compile: javac Chapter13_Lesson1.java
// Run:     java Chapter13_Lesson1

public class Chapter13_Lesson1 {
    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int p = B.length;
        int n = B[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int k = 0; k < p; k++)
                for (int j = 0; j < n; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        int q = C.length;
        double[][] O = new double[q * n][n];
        double[][] Ak = identity(n);

        for (int block = 0; block < n; block++) {
            double[][] CAk = multiply(C, Ak);
            for (int i = 0; i < q; i++)
                for (int j = 0; j < n; j++)
                    O[block * q + i][j] = CAk[i][j];
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            System.arraycopy(input[i], 0, M[i], 0, cols);

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++)
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;

            if (Math.abs(M[pivot][c]) < tol) continue;

            double[] tmp = M[r];
            M[r] = M[pivot];
            M[pivot] = tmp;

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

    static void printMatrix(double[][] M) {
        for (double[] row : M) {
            for (double x : row) System.out.printf("%10.5f ", x);
            System.out.println();
        }
    }

    static void report(String name, double[][] A, double[][] C) {
        double[][] O = observabilityMatrix(A, C);
        System.out.println("\n" + name);
        System.out.println("O_n =");
        printMatrix(O);
        System.out.println("rank(O_n) = " + rank(O, 1e-10) + " out of n = " + A.length);
    }

    public static void main(String[] args) {
        double[][] A1 = {{0.0, 1.0}, {-2.0, -3.0}};
        double[][] C1 = {{1.0, 0.0}};
        report("Example 1: position sensor, observable", A1, C1);

        double[][] A2 = {{-1.0, 0.0}, {0.0, -2.0}};
        double[][] C2 = {{1.0, 0.0}};
        report("Example 2: second state invisible, unobservable", A2, C2);
    }
}
