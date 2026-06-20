// Chapter16_Lesson1.java
// Compile: javac Chapter16_Lesson1.java
// Run:     java Chapter16_Lesson1
// Convention: p(s)=s^n+a[n-1]s^(n-1)+...+a[1]s+a[0]

public class Chapter16_Lesson1 {

    static double[][] companionA(double[] a) {
        int n = a.length;
        double[][] A = new double[n][n];

        for (int i = 0; i < n - 1; i++) {
            A[i][i + 1] = 1.0;
        }
        for (int j = 0; j < n; j++) {
            A[n - 1][j] = -a[j];
        }
        return A;
    }

    static double[] inputB(int n) {
        double[] B = new double[n];
        B[n - 1] = 1.0;
        return B;
    }

    static double[] matVec(double[][] A, double[] x) {
        int n = A.length;
        double[] y = new double[n];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                y[i] += A[i][j] * x[j];
            }
        }
        return y;
    }

    static double[][] controllabilityMatrix(double[][] A, double[] B) {
        int n = A.length;
        double[][] Q = new double[n][n];
        double[] v = B.clone();

        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                Q[i][k] = v[i];
            }
            v = matVec(A, v);
        }
        return Q;
    }

    static double determinant(double[][] input) {
        int n = input.length;
        double[][] M = new double[n][n];

        for (int i = 0; i < n; i++) {
            System.arraycopy(input[i], 0, M[i], 0, n);
        }

        double det = 1.0;
        for (int k = 0; k < n; k++) {
            int pivot = k;
            for (int i = k + 1; i < n; i++) {
                if (Math.abs(M[i][k]) > Math.abs(M[pivot][k])) {
                    pivot = i;
                }
            }

            if (Math.abs(M[pivot][k]) < 1e-12) {
                return 0.0;
            }

            if (pivot != k) {
                double[] temp = M[pivot];
                M[pivot] = M[k];
                M[k] = temp;
                det *= -1.0;
            }

            det *= M[k][k];
            double pivotValue = M[k][k];

            for (int i = k + 1; i < n; i++) {
                double factor = M[i][k] / pivotValue;
                for (int j = k; j < n; j++) {
                    M[i][j] -= factor * M[k][j];
                }
            }
        }
        return det;
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
        // Example: p(s)=s^3+4s^2+3s+2
        double[] a = {2.0, 3.0, 4.0};  // a0, a1, a2

        double[][] A = companionA(a);
        double[] B = inputB(a.length);
        double[][] Qc = controllabilityMatrix(A, B);

        printMatrix("A", A);
        System.out.print("B = [ ");
        for (double value : B) {
            System.out.print(value + " ");
        }
        System.out.println("]^T");

        printMatrix("Q_c", Qc);
        System.out.println("det(Q_c) = " + determinant(Qc));
        System.out.println("The nonzero determinant confirms controllability in CCF.");
    }
}
