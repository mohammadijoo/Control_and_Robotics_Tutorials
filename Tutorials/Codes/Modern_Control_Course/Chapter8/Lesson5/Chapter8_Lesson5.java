/*
Chapter8_Lesson5.java

Self-contained Java implementation of Phi(t)=exp(A t) using
scaling-and-squaring with a truncated Taylor series.

For production-grade scientific computing, consider libraries such as EJML,
Apache Commons Math, or ojAlgo for matrix storage and factorizations; however,
many Java libraries do not expose a direct matrix exponential, so this file
keeps the exponential implementation explicit for teaching purposes.

Compile:
  javac Chapter8_Lesson5.java
Run:
  java Chapter8_Lesson5
*/

public class Chapter8_Lesson5 {
    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] add(double[][] A, double[][] B) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] subtract(double[][] A, double[][] B) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    static double[][] scalarMultiply(double[][] A, double alpha) {
        int n = A.length, m = A[0].length;
        double[][] B = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                B[i][j] = alpha * A[i][j];
        return B;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length, p = A[0].length, m = B[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < m; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double infNorm(double[][] A) {
        double max = 0.0;
        for (int i = 0; i < A.length; i++) {
            double rowSum = 0.0;
            for (int j = 0; j < A[0].length; j++) {
                rowSum += Math.abs(A[i][j]);
            }
            max = Math.max(max, rowSum);
        }
        return max;
    }

    static double[][] expmTaylorScalingSquaring(double[][] M, int order) {
        int n = M.length;
        double mNorm = infNorm(M);
        int scale = 0;
        if (mNorm > 0.0) {
            scale = Math.max(0, (int)Math.ceil(Math.log(mNorm) / Math.log(2.0)));
        }

        double[][] AScaled = scalarMultiply(M, 1.0 / Math.pow(2.0, scale));
        double[][] E = identity(n);
        double[][] term = identity(n);

        for (int k = 1; k <= order; k++) {
            term = scalarMultiply(multiply(term, AScaled), 1.0 / k);
            E = add(E, term);
        }

        for (int i = 0; i < scale; i++) {
            E = multiply(E, E);
        }

        return E;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name + " =");
        for (double[] row : A) {
            for (double x : row) {
                System.out.printf("%14.8f ", x);
            }
            System.out.println();
        }
        System.out.println();
    }

    public static void main(String[] args) {
        double[][] A = {
            { 0.0, 1.0, 0.0 },
            { 0.0, 0.0, 1.0 },
            {-2.0,-3.0,-4.0 }
        };

        double t = 0.5;
        double s = 0.25;

        double[][] At = scalarMultiply(A, t);
        double[][] As = scalarMultiply(A, s);
        double[][] A_t_plus_s = scalarMultiply(A, t + s);

        double[][] PhiT = expmTaylorScalingSquaring(At, 45);
        double[][] PhiS = expmTaylorScalingSquaring(As, 45);
        double[][] PhiTplusS = expmTaylorScalingSquaring(A_t_plus_s, 45);

        printMatrix("A", A);
        printMatrix("Phi(t)", PhiT);

        double[][] semigroupError = subtract(multiply(PhiT, PhiS), PhiTplusS);
        System.out.println("Semigroup error ||Phi(t)Phi(s)-Phi(t+s)||_inf = "
                           + infNorm(semigroupError));

        double[][] PhiMinusT = expmTaylorScalingSquaring(scalarMultiply(A, -t), 45);
        double[][] inverseError = subtract(multiply(PhiT, PhiMinusT), identity(3));
        System.out.println("Inverse error ||Phi(t)Phi(-t)-I||_inf = "
                           + infNorm(inverseError));
    }
}
