// Chapter21_Lesson1.java
// Finite transmission-zero check using the Rosenbrock system matrix.
// This version is implemented from scratch, without external libraries.

public class Chapter21_Lesson1 {
    static double[][] rosenbrockMatrix(double[][] A, double[][] B,
                                       double[][] C, double[][] D, double s) {
        int n = A.length;
        int p = C.length;
        int m = B[0].length;
        double[][] R = new double[n + p][n + m];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                R[i][j] = -A[i][j];
                if (i == j) {
                    R[i][j] += s;
                }
            }
            for (int j = 0; j < m; j++) {
                R[i][n + j] = -B[i][j];
            }
        }

        for (int i = 0; i < p; i++) {
            for (int j = 0; j < n; j++) {
                R[n + i][j] = C[i][j];
            }
            for (int j = 0; j < m; j++) {
                R[n + i][n + j] = D[i][j];
            }
        }

        return R;
    }

    static double determinant(double[][] input) {
        int n = input.length;
        double[][] A = copy(input);
        double det = 1.0;

        for (int k = 0; k < n; k++) {
            int pivot = k;
            for (int i = k + 1; i < n; i++) {
                if (Math.abs(A[i][k]) > Math.abs(A[pivot][k])) {
                    pivot = i;
                }
            }

            if (Math.abs(A[pivot][k]) < 1e-12) {
                return 0.0;
            }

            if (pivot != k) {
                double[] tmp = A[k];
                A[k] = A[pivot];
                A[pivot] = tmp;
                det *= -1.0;
            }

            det *= A[k][k];
            double pivotValue = A[k][k];
            for (int i = k + 1; i < n; i++) {
                double factor = A[i][k] / pivotValue;
                for (int j = k; j < n; j++) {
                    A[i][j] -= factor * A[k][j];
                }
            }
        }

        return det;
    }

    static int rank(double[][] input, double tol) {
        double[][] A = copy(input);
        int rows = A.length;
        int cols = A[0].length;
        int rank = 0;
        int row = 0;

        for (int col = 0; col < cols && row < rows; col++) {
            int pivot = row;
            for (int i = row + 1; i < rows; i++) {
                if (Math.abs(A[i][col]) > Math.abs(A[pivot][col])) {
                    pivot = i;
                }
            }

            if (Math.abs(A[pivot][col]) <= tol) {
                continue;
            }

            double[] tmp = A[row];
            A[row] = A[pivot];
            A[pivot] = tmp;

            double pivotValue = A[row][col];
            for (int j = col; j < cols; j++) {
                A[row][j] /= pivotValue;
            }

            for (int i = 0; i < rows; i++) {
                if (i != row) {
                    double factor = A[i][col];
                    for (int j = col; j < cols; j++) {
                        A[i][j] -= factor * A[row][j];
                    }
                }
            }

            rank++;
            row++;
        }

        return rank;
    }

    static double[][] copy(double[][] input) {
        double[][] out = new double[input.length][input[0].length];
        for (int i = 0; i < input.length; i++) {
            System.arraycopy(input[i], 0, out[i], 0, input[i].length);
        }
        return out;
    }

    static void printMatrix(double[][] M) {
        for (double[] row : M) {
            for (double value : row) {
                System.out.printf("%12.6f ", value);
            }
            System.out.println();
        }
    }

    static double detR(double[][] A, double[][] B, double[][] C, double[][] D, double s) {
        return determinant(rosenbrockMatrix(A, B, C, D, s));
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0},
            {-2.0, -3.0}
        };
        double[][] B = {
            {0.0},
            {1.0}
        };
        double[][] C = {
            {4.0, 1.0}
        };
        double[][] D = {
            {0.0}
        };

        double z = -4.0;
        double[][] Rz = rosenbrockMatrix(A, B, C, D, z);

        System.out.println("R(-4) =");
        printMatrix(Rz);
        System.out.println("rank R(-4) = " + rank(Rz, 1e-9));
        System.out.println("normal rank = " + (A.length + B[0].length));
        System.out.println("det R(-4) = " + determinant(Rz));

        System.out.println("\nReal-line scan of det R(s):");
        double previousS = -20.0;
        double previousF = detR(A, B, C, D, previousS);

        for (double s = -19.9; s <= 20.0; s += 0.1) {
            double f = detR(A, B, C, D, s);
            if (previousF * f < 0.0 || Math.abs(f) < 1e-8) {
                System.out.printf("candidate zero near s = %.3f%n", s);
            }
            previousS = s;
            previousF = f;
        }
    }
}
