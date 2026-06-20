/*
Chapter10_Lesson4.java

Physical Interpretation: Actuator Placement and Authority
Modern Control - Chapter 10, Lesson 4

This Java example computes the reachability matrix and its numerical rank
for several actuator-placement choices in a two-degree-of-freedom system.

Compile and run:
    javac Chapter10_Lesson4.java
    java Chapter10_Lesson4
*/

public class Chapter10_Lesson4 {
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
        int kdim = B.length;
        int c = B[0].length;
        double[][] C = zeros(r, c);

        for (int i = 0; i < r; i++) {
            for (int k = 0; k < kdim; k++) {
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
        int col0 = 0;

        for (double[][] block : blocks) {
            int cols = block[0].length;
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    H[i][col0 + j] = block[i][j];
                }
            }
            col0 += cols;
        }
        return H;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Ak = identity(n);
        double[][][] blocks = new double[n][][];

        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(Ak, A);
        }
        return hstack(blocks);
    }

    static int numericalRank(double[][] input, double tol) {
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

            double[] tmp = M[pivot];
            M[pivot] = M[rank];
            M[rank] = tmp;

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

    static double frobeniusNorm(double[][] M) {
        double sum = 0.0;
        for (double[] row : M) {
            for (double value : row) sum += value * value;
        }
        return Math.sqrt(sum);
    }

    static void analyzePlacement(String name, double[][] A, double[][] B) {
        double[][] C = controllabilityMatrix(A, B);
        int rank = numericalRank(C, 1e-9);
        double score = frobeniusNorm(C);

        System.out.println("\nActuator placement: " + name);
        System.out.println("Rank of [B, AB, ..., A^(n-1)B]: " + rank + " out of " + A.length);
        System.out.printf("Frobenius norm authority score of reachability matrix: %.6f%n", score);
    }

    public static void main(String[] args) {
        double[][] A = {
            { 0.0,  0.0, 1.0, 0.0},
            { 0.0,  0.0, 0.0, 1.0},
            {-2.0,  1.0,-0.08,0.0},
            { 1.0, -2.0, 0.0,-0.08}
        };

        double[][] BMass1 = {
            {0.0},
            {0.0},
            {1.0},
            {0.0}
        };

        double[][] BMass2 = {
            {0.0},
            {0.0},
            {0.0},
            {1.0}
        };

        double[][] BCommon = {
            {0.0},
            {0.0},
            {1.0},
            {1.0}
        };

        double[][] BBoth = {
            {0.0, 0.0},
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 1.0}
        };

        analyzePlacement("force on mass 1 only", A, BMass1);
        analyzePlacement("force on mass 2 only", A, BMass2);
        analyzePlacement("same force on both masses", A, BCommon);
        analyzePlacement("independent forces on both masses", A, BBoth);
    }
}
