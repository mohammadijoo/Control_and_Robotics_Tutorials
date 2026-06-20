// Chapter25_Lesson5.java
/*
Structural Constraints: Limited Actuators and Sparse Feedback

Compile and run:
    javac Chapter25_Lesson5.java
    java Chapter25_Lesson5

This pure Java version implements basic matrix multiplication, controllability
matrix construction, numerical rank by Gaussian elimination, and Euler
simulation of the closed loop.
*/

public class Chapter25_Lesson5 {
    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length;
        int p = B[0].length;
        int m = B.length;
        double[][] C = new double[n][p];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < m; k++) {
                for (int j = 0; j < p; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] subtract(double[][] A, double[][] B) {
        int n = A.length;
        int m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) {
            I[i][i] = 1.0;
        }
        return I;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        int m = B[0].length;
        double[][] C = new double[n][n * m];
        double[][] Ap = identity(n);
        for (int block = 0; block < n; block++) {
            double[][] ApB = multiply(Ap, B);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    C[i][block * m + j] = ApB[i][j];
                }
            }
            Ap = multiply(Ap, A);
        }
        return C;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] A = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(input[i], 0, A[i], 0, cols);
        }

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(A[i][c]) > Math.abs(A[pivot][c])) {
                    pivot = i;
                }
            }
            if (Math.abs(A[pivot][c]) <= tol) {
                continue;
            }
            double[] temp = A[r];
            A[r] = A[pivot];
            A[pivot] = temp;

            double div = A[r][c];
            for (int j = c; j < cols; j++) {
                A[r][j] /= div;
            }
            for (int i = 0; i < rows; i++) {
                if (i != r) {
                    double factor = A[i][c];
                    for (int j = c; j < cols; j++) {
                        A[i][j] -= factor * A[r][j];
                    }
                }
            }
            r++;
        }
        return r;
    }

    static double[][] applyMask(double[][] K, double[][] mask) {
        int rows = K.length;
        int cols = K[0].length;
        double[][] S = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                S[i][j] = K[i][j] * mask[i][j];
            }
        }
        return S;
    }

    static double[] matVec(double[][] A, double[] x) {
        int n = A.length;
        int m = A[0].length;
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                y[i] += A[i][j] * x[j];
            }
        }
        return y;
    }

    static double[] simulate(double[][] Acl, double[] x0, double dt, int steps) {
        double[] x = x0.clone();
        for (int k = 0; k < steps; k++) {
            double[] dx = matVec(Acl, x);
            for (int i = 0; i < x.length; i++) {
                x[i] += dt * dx[i];
            }
        }
        return x;
    }

    static void printMatrix(String label, double[][] M) {
        System.out.println(label);
        for (double[] row : M) {
            for (double value : row) {
                System.out.printf("%10.4f ", value);
            }
            System.out.println();
        }
    }

    static void printVector(String label, double[] v) {
        System.out.print(label);
        for (double value : v) {
            System.out.printf("%10.4f ", value);
        }
        System.out.println();
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0,  1.0,  0.0,  0.0},
            {-2.0, -0.25, 0.7, 0.0},
            {0.0,  0.0,  0.0,  1.0},
            {0.6,  0.0, -1.5, -0.20}
        };

        double[][] B = {
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 0.0},
            {0.0, 1.0}
        };

        double[][] Kdense = {
            {3.0, 2.0, 0.9, 0.4},
            {0.6, 0.3, 2.6, 1.8}
        };

        double[][] mask = {
            {1.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 1.0}
        };

        double[][] C = controllabilityMatrix(A, B);
        System.out.println("rank(C) = " + rank(C, 1.0e-9) + " / " + A.length);

        double[][] Ksparse = applyMask(Kdense, mask);
        printMatrix("\nDense K:", Kdense);
        printMatrix("\nSparse K:", Ksparse);

        double[][] AclDense = subtract(A, multiply(B, Kdense));
        double[][] AclSparse = subtract(A, multiply(B, Ksparse));

        double[] x0 = {1.0, 0.0, -0.7, 0.2};
        double[] xfDense = simulate(AclDense, x0, 0.001, 10000);
        double[] xfSparse = simulate(AclSparse, x0, 0.001, 10000);

        printVector("\nFinal dense state:  ", xfDense);
        printVector("Final sparse state: ", xfSparse);

        System.out.println("\nUse EJML or Apache Commons Math if eigenvalues and Riccati solvers");
        System.out.println("are needed in a larger Java modern-control project.");
    }
}
