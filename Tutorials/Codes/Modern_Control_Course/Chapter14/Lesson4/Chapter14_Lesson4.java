// Chapter14_Lesson4.java
// Observability in canonical (observable) forms from scratch.
// Compile: javac Chapter14_Lesson4.java
// Run:     java Chapter14_Lesson4

public class Chapter14_Lesson4 {
    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] identity(int n) {
        double[][] I = zeros(n, n);
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int m = A[0].length;
        int c = B[0].length;
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++) {
            for (int k = 0; k < m; k++) {
                for (int j = 0; j < c; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] observableCompanion(double[] den) {
        // den = [a0, a1, ..., a_{n-1}]
        int n = den.length;
        double[][] A = zeros(n, n);
        for (int i = 0; i < n; i++) A[i][0] = -den[n - 1 - i];
        for (int i = 0; i < n - 1; i++) A[i][i + 1] = 1.0;
        return A;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        double[][] O = zeros(n, n);
        double[][] Ak = identity(n);
        for (int block = 0; block < n; block++) {
            double[][] row = multiply(C, Ak);
            for (int j = 0; j < n; j++) O[block][j] = row[0][j];
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static int rank(double[][] input, double tol) {
        int m = input.length;
        int n = input[0].length;
        double[][] A = new double[m][n];
        for (int i = 0; i < m; i++) System.arraycopy(input[i], 0, A[i], 0, n);

        int rank = 0;
        for (int col = 0; col < n && rank < m; col++) {
            int pivot = rank;
            for (int i = rank + 1; i < m; i++) {
                if (Math.abs(A[i][col]) > Math.abs(A[pivot][col])) pivot = i;
            }
            if (Math.abs(A[pivot][col]) <= tol) continue;
            double[] temp = A[pivot];
            A[pivot] = A[rank];
            A[rank] = temp;

            double div = A[rank][col];
            for (int j = col; j < n; j++) A[rank][j] /= div;
            for (int i = 0; i < m; i++) {
                if (i == rank) continue;
                double factor = A[i][col];
                for (int j = col; j < n; j++) A[i][j] -= factor * A[rank][j];
            }
            rank++;
        }
        return rank;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double v : row) System.out.printf("%12.6f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        // Denominator: s^3 + 6s^2 + 11s + 6
        double[] den = {6.0, 11.0, 6.0};
        double[][] A = observableCompanion(den);
        double[][] C = {{1.0, 0.0, 0.0}};
        double[][] O = observabilityMatrix(A, C);

        printMatrix("A_o", A);
        printMatrix("C_o", C);
        printMatrix("O", O);
        System.out.println("rank(O) = " + rank(O, 1e-10) + " out of " + A.length);
    }
}
