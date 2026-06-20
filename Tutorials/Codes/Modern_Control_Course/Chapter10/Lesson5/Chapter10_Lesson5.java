// Chapter10_Lesson5.java
// Modern Control - Chapter 10, Lesson 5
// Controllability examples using from-scratch matrix operations.
// Compile:
//     javac Chapter10_Lesson5.java
// Run:
//     java Chapter10_Lesson5

public class Chapter10_Lesson5 {
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

    static double[][] hstack(double[][][] blocks) {
        int rows = blocks[0].length;
        int totalCols = 0;
        for (double[][] block : blocks) totalCols += block[0].length;

        double[][] H = zeros(rows, totalCols);
        int offset = 0;
        for (double[][] block : blocks) {
            int cols = block[0].length;
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    H[i][offset + j] = block[i][j];
                }
            }
            offset += cols;
        }
        return H;
    }

    static int rankGaussian(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(input[i], 0, M[i], 0, cols);
        }

        int rank = 0;
        for (int col = 0; col < cols && rank < rows; col++) {
            int pivot = rank;
            for (int r = rank + 1; r < rows; r++) {
                if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) {
                    pivot = r;
                }
            }

            if (Math.abs(M[pivot][col]) <= tol) continue;

            double[] temp = M[rank];
            M[rank] = M[pivot];
            M[pivot] = temp;

            double pivotValue = M[rank][col];
            for (int j = col; j < cols; j++) M[rank][j] /= pivotValue;

            for (int r = 0; r < rows; r++) {
                if (r == rank) continue;
                double factor = M[r][col];
                for (int j = col; j < cols; j++) {
                    M[r][j] -= factor * M[rank][j];
                }
            }
            rank++;
        }
        return rank;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(Ak, A);
        }
        return hstack(blocks);
    }

    static void printMatrix(double[][] M) {
        for (double[] row : M) {
            for (double value : row) {
                System.out.printf("%10.4f ", value);
            }
            System.out.println();
        }
    }

    static void analyze(String name, double[][] A, double[][] B) {
        double[][] C = controllabilityMatrix(A, B);
        int r = rankGaussian(C, 1e-10);
        int n = A.length;

        System.out.println("\n" + name);
        System.out.println("-".repeat(name.length()));
        System.out.println("Controllability matrix:");
        printMatrix(C);
        System.out.println("rank(C) = " + r + " out of n = " + n);
        System.out.println("Conclusion: " + (r == n ? "controllable" : "uncontrollable"));
    }

    public static void main(String[] args) {
        double[][] A1 = {{0, 1},
                         {0, 0}};
        double[][] B1 = {{0},
                         {1}};
        analyze("Example 1: double integrator", A1, B1);

        double[][] A2 = {{0, 1, 0, 0},
                         {0, 0, 0, 0},
                         {0, 0, 0, 1},
                         {0, 0, 0, 0}};
        double[][] B2 = {{0},
                         {1},
                         {0},
                         {0}};
        analyze("Example 2: two decoupled masses, one actuator", A2, B2);

        double m1 = 1.0, m2 = 1.0, k1 = 1.0, k2 = 1.2, kc = 0.8, c1 = 0.1, c2 = 0.2;
        double[][] A3 = {{0, 1, 0, 0},
                         {-(k1 + kc) / m1, -c1 / m1, kc / m1, 0},
                         {0, 0, 0, 1},
                         {kc / m2, 0, -(k2 + kc) / m2, -c2 / m2}};
        double[][] B3 = {{0},
                         {1.0 / m1},
                         {0},
                         {0}};
        analyze("Example 3: coupled two-mass oscillator", A3, B3);

        double[][] A4 = {{-1, 0, 0},
                         {0, -2, 0},
                         {0, 0, -3}};
        double[][] B4 = {{1},
                         {0},
                         {1}};
        analyze("Example 4: diagonal system with one unactuated mode", A4, B4);
    }
}
