// Chapter11_Lesson3.java
// Controllability in companion controllable canonical form.
// Compile: javac Chapter11_Lesson3.java
// Run:     java Chapter11_Lesson3

public class Chapter11_Lesson3 {
    static double[][] zeros(int rows, int cols) {
        return new double[rows][cols];
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

    static double[][] companionA(double[] coeffs) {
        int n = coeffs.length;
        double[][] A = zeros(n, n);
        for (int i = 0; i < n - 1; i++) A[i][i + 1] = 1.0;
        for (int j = 0; j < n; j++) A[n - 1][j] = -coeffs[j];
        return A;
    }

    static double[][] companionB(int n) {
        double[][] B = zeros(n, 1);
        B[n - 1][0] = 1.0;
        return B;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Q = zeros(n, n);
        double[][] Ak = identity(n);
        for (int col = 0; col < n; col++) {
            double[][] block = multiply(Ak, B);
            for (int row = 0; row < n; row++) Q[row][col] = block[row][0];
            Ak = multiply(Ak, A);
        }
        return Q;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(input[i], 0, M[i], 0, cols);
        }

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            }
            if (Math.abs(M[pivot][c]) <= tol) continue;

            double[] tmp = M[pivot];
            M[pivot] = M[r];
            M[r] = tmp;

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

    static void printMatrix(double[][] M, String name) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double x : row) System.out.printf("%12.6f ", x);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[] coeffs = {6.0, 11.0, 6.0}; // p(s)=s^3+6s^2+11s+6
        double[][] A = companionA(coeffs);
        double[][] B = companionB(coeffs.length);
        double[][] Q = controllabilityMatrix(A, B);

        printMatrix(A, "A_c");
        printMatrix(B, "B_c");
        printMatrix(Q, "Q_c");
        System.out.println("rank(Q_c) = " + rank(Q, 1e-10));
        System.out.println("The companion pair is controllable when rank(Q_c) equals n.");
    }
}
