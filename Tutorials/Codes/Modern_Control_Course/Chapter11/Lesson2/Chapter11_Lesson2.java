// Chapter11_Lesson2.java
// PBH controllability test for real-eigenvalue examples using Apache Commons Math.
// Compile example:
//   javac -cp commons-math3-3.6.1.jar Chapter11_Lesson2.java
// Run example:
//   java -cp .:commons-math3-3.6.1.jar Chapter11_Lesson2

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

public class Chapter11_Lesson2 {
    static double[][] hstackPbh(double[][] A, double[][] B, double lambda) {
        int n = A.length;
        int m = B[0].length;
        double[][] M = new double[n][n + m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                M[i][j] = (i == j ? lambda : 0.0) - A[i][j];
            }
            for (int j = 0; j < m; j++) {
                M[i][n + j] = B[i][j];
            }
        }
        return M;
    }

    static int matrixRank(double[][] M) {
        RealMatrix R = new Array2DRowRealMatrix(M);
        return new SingularValueDecomposition(R).getRank();
    }

    static List<Double> uniqueRealEigenvalues(double[][] A, double tol) {
        RealMatrix R = new Array2DRowRealMatrix(A);
        EigenDecomposition ed = new EigenDecomposition(R);
        List<Double> values = new ArrayList<Double>();
        for (int i = 0; i < A.length; i++) {
            double real = ed.getRealEigenvalue(i);
            double imag = ed.getImagEigenvalue(i);
            if (Math.abs(imag) > tol) {
                throw new IllegalArgumentException(
                    "This compact Java demo handles real eigenvalues only. " +
                    "Use a complex linear algebra package for lambda = " + real + " + " + imag + "i."
                );
            }
            boolean found = false;
            for (double old : values) {
                if (Math.abs(real - old) <= tol) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                values.add(real);
            }
        }
        return values;
    }

    static boolean pbhRankTest(double[][] A, double[][] B) {
        int n = A.length;
        boolean ok = true;
        for (double lambda : uniqueRealEigenvalues(A, 1e-8)) {
            double[][] M = hstackPbh(A, B, lambda);
            int rank = matrixRank(M);
            System.out.printf("lambda = %.6f, rank([lambda I - A, B]) = %d/%d%n", lambda, rank, n);
            if (rank != n) {
                ok = false;
            }
        }
        return ok;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        int m = B[0].length;
        double[][] C = new double[n][n * m];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            double[][] block = multiply(Ak, B);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    C[i][k * m + j] = block[i][j];
                }
            }
            Ak = multiply(Ak, A);
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

    static double[][] multiply(double[][] X, double[][] Y) {
        int r = X.length;
        int c = Y[0].length;
        int inner = Y.length;
        double[][] Z = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                double sum = 0.0;
                for (int k = 0; k < inner; k++) {
                    sum += X[i][k] * Y[k][j];
                }
                Z[i][j] = sum;
            }
        }
        return Z;
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 0.0, 0.0},
            {0.0, -1.0, 0.0},
            {0.0, 0.0, -2.0}
        };
        double[][] Bbad = {{0.0}, {1.0}, {1.0}};
        double[][] Bgood = {{1.0}, {1.0}, {1.0}};

        System.out.println("Example 1: unactuated first mode");
        System.out.println("Kalman rank = " + matrixRank(controllabilityMatrix(A, Bbad)));
        System.out.println("PBH controllable? " + pbhRankTest(A, Bbad));

        System.out.println("\nExample 2: all modes actuated");
        System.out.println("Kalman rank = " + matrixRank(controllabilityMatrix(A, Bgood)));
        System.out.println("PBH controllable? " + pbhRankTest(A, Bgood));
    }
}
