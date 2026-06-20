
public class TwoTaskHierarchy {

    // Multiply A (m x n) by B (n x p)
    static double[][] matMul(double[][] A, double[][] B) {
        int m = A.length;
        int n = A[0].length;
        int p = B[0].length;
        double[][] C = new double[m][p];
        for (int i = 0; i < m; ++i) {
            for (int k = 0; k < n; ++k) {
                for (int j = 0; j < p; ++j) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    // Transpose of A
    static double[][] transpose(double[][] A) {
        int m = A.length;
        int n = A[0].length;
        double[][] At = new double[n][m];
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                At[j][i] = A[i][j];
            }
        }
        return At;
    }

    // Very naive matrix inverse for small matrices using Gauss-Jordan
    static double[][] invert(double[][] A) {
        int n = A.length;
        double[][] B = new double[n][2*n];
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                B[i][j] = A[i][j];
            }
            B[i][n + i] = 1.0;
        }
        for (int i = 0; i < n; ++i) {
            double pivot = B[i][i];
            for (int j = 0; j < 2*n; ++j) {
                B[i][j] /= pivot;
            }
            for (int k = 0; k < n; ++k) {
                if (k == i) continue;
                double factor = B[k][i];
                for (int j = 0; j < 2*n; ++j) {
                    B[k][j] -= factor * B[i][j];
                }
            }
        }
        double[][] Inv = new double[n][n];
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                Inv[i][j] = B[i][n + j];
            }
        }
        return Inv;
    }

    static double[][] pseudoinverse(double[][] J) {
        // J# = J^T (J J^T)^(-1)
        double[][] Jt = transpose(J);
        double[][] JJt = matMul(J, Jt);
        double[][] JJtInv = invert(JJt);
        return matMul(Jt, JJtInv);
    }

    static double[] twoTaskHierarchy(double[][] J1, double[] dx1,
                                     double[][] J2, double[] dx2) {
        int n = J1[0].length;
        double[][] I = new double[n][n];
        for (int i = 0; i < n; ++i) I[i][i] = 1.0;

        double[][] J1_pinv = pseudoinverse(J1);
        double[] dq1 = matVecMul(J1_pinv, dx1);
        double[][] N1 = matSub(I, matMul(J1_pinv, J1));

        double[][] J2_bar = matMul(J2, N1);
        double[][] J2_bar_pinv = pseudoinverse(J2_bar);
        double[] J2dq1 = matVecMul(J2, dq1);
        double[] dx2_tilde = vecSub(dx2, J2dq1);
        double[] dq2_null = matVecMul(J2_bar_pinv, dx2_tilde);

        double[] dq = vecAdd(dq1, matVecMul(N1, dq2_null));
        return dq;
    }

    // Helper functions matVecMul, vecAdd, vecSub are omitted for brevity
    // but follow straightforward double-loop implementations.
}
