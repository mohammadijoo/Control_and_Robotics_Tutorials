// Chapter11_Lesson1.java
// Kalman controllability matrix and rank condition using plain Java arrays.

public class Chapter11_Lesson1 {

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
        int k = A[0].length;
        int c = B[0].length;
        if (B.length != k) throw new IllegalArgumentException("Inner dimensions do not match.");

        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                for (int p = 0; p < k; p++) {
                    C[i][j] += A[i][p] * B[p][j];
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

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        if (A[0].length != n) throw new IllegalArgumentException("A must be square.");
        if (B.length != n) throw new IllegalArgumentException("B must have the same row count as A.");

        double[][][] blocks = new double[n][][];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(A, Ak);
        }
        return hstack(blocks);
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
            for (int i = rank + 1; i < rows; i++) {
                if (Math.abs(M[i][col]) > Math.abs(M[pivot][col])) pivot = i;
            }

            if (Math.abs(M[pivot][col]) <= tol) continue;

            double[] temp = M[rank];
            M[rank] = M[pivot];
            M[pivot] = temp;

            double piv = M[rank][col];
            for (int j = col; j < cols; j++) M[rank][j] /= piv;

            for (int i = 0; i < rows; i++) {
                if (i == rank) continue;
                double factor = M[i][col];
                for (int j = col; j < cols; j++) {
                    M[i][j] -= factor * M[rank][j];
                }
            }
            rank++;
        }
        return rank;
    }

    static void printMatrix(double[][] M) {
        for (double[] row : M) {
            for (double v : row) {
                System.out.printf("%12.6f ", v);
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = {
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0},
                {-6.0, -11.0, -6.0}
        };

        double[][] B = {
                {0.0},
                {0.0},
                {1.0}
        };

        double[][] Ck = controllabilityMatrix(A, B);
        int rank = rankGaussian(Ck, 1e-10);

        System.out.println("Kalman controllability matrix C_K:");
        printMatrix(Ck);
        System.out.println("rank(C_K) = " + rank);
        System.out.println("controllable = " + (rank == A.length));
    }
}
