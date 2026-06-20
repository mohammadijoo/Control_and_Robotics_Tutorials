// Chapter13_Lesson5.java
// Observability examples using plain Java arrays.
// Compile: javac Chapter13_Lesson5.java
// Run:     java Chapter13_Lesson5

public class Chapter13_Lesson5 {
    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int p = A[0].length;
        int n = B[0].length;
        double[][] R = new double[m][n];

        for (int i = 0; i < m; ++i) {
            for (int k = 0; k < p; ++k) {
                for (int j = 0; j < n; ++j) {
                    R[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return R;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; ++i) I[i][i] = 1.0;
        return I;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        int p = C.length;
        double[][] O = new double[p * n][n];
        double[][] Ak = identity(n);

        for (int k = 0; k < n; ++k) {
            double[][] block = multiply(C, Ak);
            for (int i = 0; i < p; ++i) {
                for (int j = 0; j < n; ++j) {
                    O[k * p + i][j] = block[i][j];
                }
            }
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];

        for (int i = 0; i < rows; ++i) {
            System.arraycopy(input[i], 0, M[i], 0, cols);
        }

        int r = 0;
        for (int col = 0; col < cols && r < rows; ++col) {
            int pivot = r;
            for (int i = r + 1; i < rows; ++i) {
                if (Math.abs(M[i][col]) > Math.abs(M[pivot][col])) pivot = i;
            }
            if (Math.abs(M[pivot][col]) <= tol) continue;

            double[] temp = M[pivot];
            M[pivot] = M[r];
            M[r] = temp;

            double div = M[r][col];
            for (int j = col; j < cols; ++j) M[r][j] /= div;

            for (int i = 0; i < rows; ++i) {
                if (i == r) continue;
                double factor = M[i][col];
                for (int j = col; j < cols; ++j) M[i][j] -= factor * M[r][j];
            }
            ++r;
        }
        return r;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (int i = 0; i < M.length; ++i) {
            for (int j = 0; j < M[0].length; ++j) {
                System.out.printf("%12.6f ", M[i][j]);
            }
            System.out.println();
        }
    }

    static void classify(String name, double[][] A, double[][] C) {
        double[][] O = observabilityMatrix(A, C);
        int r = rank(O, 1e-10);
        int n = A.length;

        System.out.println("\n======================================================================");
        System.out.println(name);
        printMatrix("A", A);
        printMatrix("C", C);
        printMatrix("O", O);
        System.out.println("rank(O) = " + r + " out of n = " + n);
        if (r == n) {
            System.out.println("Conclusion: observable.");
        } else {
            System.out.println("Conclusion: unobservable. Inspect hidden modes for detectability.");
        }
    }

    public static void main(String[] args) {
        double[][] A1 = {{0.0, 1.0}, {-2.0, -3.0}};
        double[][] C1 = {{1.0, 0.0}};
        classify("Example 1: observable second-order system", A1, C1);

        double[][] A2 = {{-1.0, 0.0}, {0.0, 2.0}};
        double[][] C2 = {{1.0, 0.0}};
        classify("Example 2: unobservable and not detectable", A2, C2);

        double[][] A3 = {{-1.0, 0.0}, {0.0, -4.0}};
        double[][] C3 = {{1.0, 0.0}};
        classify("Example 3: unobservable but detectable", A3, C3);

        double[][] A4 = {{0.0, 1.0}, {-6.0, -5.0}};
        double[][] C4a = {{0.0, 1.0}};
        double[][] C4b = {{1.0, 0.0}};
        classify("Example 4a: velocity sensor", A4, C4a);
        classify("Example 4b: position sensor", A4, C4b);
    }
}
