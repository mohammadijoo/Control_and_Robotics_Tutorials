/*
Chapter22_Lesson4.java
Requirements for Implementing State Feedback

Compile:
  javac Chapter22_Lesson4.java
Run:
  java Chapter22_Lesson4
*/

public class Chapter22_Lesson4 {
    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int c = B[0].length;
        int inner = B.length;
        double[][] C = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                for (int k = 0; k < inner; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] subtract(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }
        return C;
    }

    static int rank(double[][] input, double tol) {
        double[][] M = copy(input);
        int rows = M.length;
        int cols = M[0].length;
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

            double piv = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= piv;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++) M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] copy(double[][] A) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            System.arraycopy(A[i], 0, C[i], 0, A[0].length);
        }
        return C;
    }

    static double[][] controllabilityMatrix2x2(double[][] A, double[][] B) {
        double[][] AB = multiply(A, B);
        return new double[][]{
            {B[0][0], AB[0][0]},
            {B[1][0], AB[1][0]}
        };
    }

    static double[] eigenvalues2x2Real(double[][] A) {
        double tr = A[0][0] + A[1][1];
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        double disc = tr * tr - 4.0 * det;
        if (disc < 0.0) {
            throw new IllegalArgumentException("This simple demo expects real eigenvalues.");
        }
        double root = Math.sqrt(disc);
        return new double[]{(tr + root) / 2.0, (tr - root) / 2.0};
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name);
        for (double[] row : M) {
            for (double v : row) System.out.printf("%12.6f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-2.0, -3.0}};
        double[][] B = {{0.0}, {1.0}};
        double[][] K = {{4.0, 2.0}};

        double[][] Wc = controllabilityMatrix2x2(A, B);
        printMatrix("Controllability matrix Wc:", Wc);
        System.out.println("rank(Wc) = " + rank(Wc, 1e-10) + " required = 2");

        double[][] Acl = subtract(A, multiply(B, K));
        printMatrix("\nClosed-loop matrix Acl = A - BK:", Acl);

        double[] eigs = eigenvalues2x2Real(Acl);
        System.out.printf("closed-loop eigenvalues: %.6f, %.6f%n", eigs[0], eigs[1]);

        double[] x = {1.0, 0.0};
        double dt = 0.01;
        int steps = (int)(5.0 / dt);

        for (int k = 0; k < steps; k++) {
            double dx0 = Acl[0][0] * x[0] + Acl[0][1] * x[1];
            double dx1 = Acl[1][0] * x[0] + Acl[1][1] * x[1];
            x[0] += dt * dx0;
            x[1] += dt * dx1;
        }

        System.out.printf("Final simulated state: [%.6f, %.6f]%n", x[0], x[1]);
    }
}
