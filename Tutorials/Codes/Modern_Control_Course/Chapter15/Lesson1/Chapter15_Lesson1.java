// Chapter15_Lesson1.java
// Finite-horizon observability Gramian with only core Java.
//
// The code integrates:
//   dW/dt = A^T W + W A + C^T C, W(0)=0.
// For production-scale work, use EJML, Apache Commons Math, or ojAlgo.

public class Chapter15_Lesson1 {
    static double[][] transpose(double[][] A) {
        int m = A.length;
        int n = A[0].length;
        double[][] T = new double[n][m];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                T[j][i] = A[i][j];
            }
        }
        return T;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int p = A[0].length;
        int n = B[0].length;
        double[][] C = new double[m][n];

        for (int i = 0; i < m; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < n; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] add(double[][] A, double[][] B) {
        int m = A.length;
        int n = A[0].length;
        double[][] C = new double[m][n];

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        int m = A.length;
        int n = A[0].length;
        double[][] C = new double[m][n];

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                C[i][j] = s * A[i][j];
            }
        }
        return C;
    }

    static double[][] rhs(double[][] A, double[][] Q, double[][] W) {
        double[][] AT = transpose(A);
        return add(add(multiply(AT, W), multiply(W, A)), Q);
    }

    static double[][] finiteObservabilityGramianRK4(double[][] A, double[][] C, double T, int steps) {
        int n = A.length;
        double h = T / steps;
        double[][] W = new double[n][n];
        double[][] Q = multiply(transpose(C), C);

        for (int k = 0; k < steps; k++) {
            double[][] K1 = rhs(A, Q, W);
            double[][] K2 = rhs(A, Q, add(W, scale(K1, 0.5 * h)));
            double[][] K3 = rhs(A, Q, add(W, scale(K2, 0.5 * h)));
            double[][] K4 = rhs(A, Q, add(W, scale(K3, h)));

            double[][] increment = scale(add(add(K1, scale(K2, 2.0)), add(scale(K3, 2.0), K4)), h / 6.0);
            W = add(W, increment);
        }

        return W;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name);
        for (double[] row : A) {
            for (double value : row) {
                System.out.printf("% .8f ", value);
            }
            System.out.println();
        }
        System.out.println();
    }

    static double quadraticForm(double[] x, double[][] W) {
        double value = 0.0;
        for (int i = 0; i < x.length; i++) {
            for (int j = 0; j < x.length; j++) {
                value += x[i] * W[i][j] * x[j];
            }
        }
        return value;
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0},
            {-2.0, -3.0}
        };

        double[][] C = {
            {1.0, 0.0}
        };

        double T = 4.0;
        double[][] W = finiteObservabilityGramianRK4(A, C, T, 20000);

        printMatrix("Finite-horizon observability Gramian W_o(T):", W);

        double[] x0 = {1.0, -0.5};
        System.out.printf("Output energy x0^T W_o(T) x0 = %.10f%n", quadraticForm(x0, W));
    }
}
