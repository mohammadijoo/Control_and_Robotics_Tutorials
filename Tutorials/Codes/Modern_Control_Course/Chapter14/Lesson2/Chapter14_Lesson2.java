/*
Chapter14_Lesson2.java
PBH Test for Observability using Apache Commons Math for eigenvalues
and a from-scratch Gaussian-elimination rank routine.

Compile example:
    javac -cp commons-math3-3.6.1.jar Chapter14_Lesson2.java
Run example:
    java -cp .:commons-math3-3.6.1.jar Chapter14_Lesson2
*/

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

public class Chapter14_Lesson2 {
    static double[][] vstack(double[][] top, double[][] bottom) {
        int rows = top.length + bottom.length;
        int cols = top[0].length;
        double[][] out = new double[rows][cols];
        for (int i = 0; i < top.length; i++) {
            System.arraycopy(top[i], 0, out[i], 0, cols);
        }
        for (int i = 0; i < bottom.length; i++) {
            System.arraycopy(bottom[i], 0, out[top.length + i], 0, cols);
        }
        return out;
    }

    static int rankReal(double[][] input, double tol) {
        int m = input.length;
        int n = input[0].length;
        double[][] a = new double[m][n];
        for (int i = 0; i < m; i++) {
            System.arraycopy(input[i], 0, a[i], 0, n);
        }

        int rank = 0;
        int row = 0;
        for (int col = 0; col < n && row < m; col++) {
            int pivot = row;
            for (int i = row + 1; i < m; i++) {
                if (Math.abs(a[i][col]) > Math.abs(a[pivot][col])) pivot = i;
            }
            if (Math.abs(a[pivot][col]) <= tol) continue;

            double[] temp = a[row];
            a[row] = a[pivot];
            a[pivot] = temp;

            double pivotValue = a[row][col];
            for (int j = col; j < n; j++) a[row][j] /= pivotValue;

            for (int i = 0; i < m; i++) {
                if (i == row) continue;
                double factor = a[i][col];
                for (int j = col; j < n; j++) a[i][j] -= factor * a[row][j];
            }
            row++;
            rank++;
        }
        return rank;
    }

    static int rankComplexByRealification(double[][] real, double[][] imag, double tol) {
        int m = real.length;
        int n = real[0].length;
        double[][] R = new double[2 * m][2 * n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                R[i][j] = real[i][j];
                R[i][j + n] = -imag[i][j];
                R[i + m][j] = imag[i][j];
                R[i + m][j + n] = real[i][j];
            }
        }
        return rankReal(R, tol);
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        int p = C.length;
        double[][] O = new double[p * n][n];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            double[][] block = multiply(C, Ak);
            for (int i = 0; i < p; i++) {
                System.arraycopy(block[i], 0, O[k * p + i], 0, n);
            }
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static boolean pbhObservable(double[][] A, double[][] C, double tol) {
        int n = A.length;
        int p = C.length;
        RealMatrix Am = new Array2DRowRealMatrix(A);
        EigenDecomposition eig = new EigenDecomposition(Am);
        boolean observable = true;

        for (int k = 0; k < n; k++) {
            double lr = eig.getRealEigenvalue(k);
            double li = eig.getImagEigenvalue(k);

            double[][] real = new double[n + p][n];
            double[][] imag = new double[n + p][n];

            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    real[i][j] = -A[i][j];
                    imag[i][j] = 0.0;
                }
                real[i][i] += lr;
                imag[i][i] += li;
            }
            for (int i = 0; i < p; i++) {
                for (int j = 0; j < n; j++) {
                    real[n + i][j] = C[i][j];
                    imag[n + i][j] = 0.0;
                }
            }

            int realifiedRank = rankComplexByRealification(real, imag, tol);
            boolean passed = (realifiedRank == 2 * n);
            observable = observable && passed;
            System.out.printf("lambda = %.6f%+.6fi, realified rank = %d, PBH mode passed = %b%n",
                    lr, li, realifiedRank, passed);
        }
        return observable;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int n = B[0].length;
        int r = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0.0;
                for (int k = 0; k < r; k++) sum += A[i][k] * B[k][j];
                C[i][j] = sum;
            }
        }
        return C;
    }

    public static void main(String[] args) {
        double[][] A1 = {{0.0, 1.0}, {-2.0, -3.0}};
        double[][] C1 = {{1.0, 0.0}};

        double[][] A2 = {{-1.0, 0.0}, {0.0, -2.0}};
        double[][] C2 = {{1.0, 0.0}};

        double[][][] As = {A1, A2};
        double[][][] Cs = {C1, C2};

        for (int i = 0; i < As.length; i++) {
            System.out.println("================ Example " + (i + 1) + " ================");
            int rankO = rankReal(observabilityMatrix(As[i], Cs[i]), 1e-9);
            System.out.println("Rank of observability matrix = " + rankO);
            boolean ok = pbhObservable(As[i], Cs[i], 1e-9);
            System.out.println("Observable by PBH = " + ok);
        }
    }
}
