// Chapter10_Lesson2.java
// Reachable States and Reachable Subspace for continuous-time LTI systems
//
// This from-scratch Java example builds the algebraic reachable subspace matrix
//   R = [B, AB, A^2B, ..., A^(n-1)B]
// and estimates the dimension of its column space using Gaussian elimination.
//
// Compile and run:
//   javac Chapter10_Lesson2.java
//   java Chapter10_Lesson2

public class Chapter10_Lesson2 {
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
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                for (int ell = 0; ell < k; ell++) {
                    C[i][j] += A[i][ell] * B[ell][j];
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

    static double[][] reachabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Ak = identity(n);
        double[][][] blocks = new double[n][][];
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(Ak, A);
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
            for (int r = rank + 1; r < rows; r++) {
                if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) pivot = r;
            }
            if (Math.abs(M[pivot][col]) <= tol) continue;

            double[] temp = M[pivot];
            M[pivot] = M[rank];
            M[rank] = temp;

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

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double value : row) {
                System.out.printf("%12.6f ", value);
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {-2.0, -3.0, 0.0},
            {0.0, 0.0, -1.0}
        };

        double[][] B = {
            {0.0},
            {1.0},
            {0.0}
        };

        double[][] R = reachabilityMatrix(A, B);
        printMatrix("A", A);
        printMatrix("B", B);
        printMatrix("R = [B AB A^2B]", R);

        int reachableDimension = rankGaussian(R, 1e-10);
        System.out.println("\nEstimated dimension of reachable subspace: " + reachableDimension);

        if (reachableDimension < A.length) {
            System.out.println("Some state directions are not reachable from the actuator.");
        } else {
            System.out.println("The reachable subspace is the whole state space.");
        }
    }
}
