/*
Chapter8_Lesson2.java
Modern Control — Chapter 8, Lesson 2
Semigroup Property and Inverse of the State Transition Matrix Phi(t)

Compile and run:
  javac Chapter8_Lesson2.java
  java Chapter8_Lesson2
*/

public class Chapter8_Lesson2 {
    static double[][] identity2() {
        return new double[][]{{1.0, 0.0}, {0.0, 1.0}};
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] subtract(double[][] A, double[][] B) {
        return add(A, scale(B, -1.0));
    }

    static double[][] scale(double[][] A, double c) {
        double[][] B = new double[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                B[i][j] = c * A[i][j];
        return B;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = new double[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[] multiply(double[][] A, double[] x) {
        return new double[]{A[0][0] * x[0] + A[0][1] * x[1],
                            A[1][0] * x[0] + A[1][1] * x[1]};
    }

    static double[] subtract(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1]};
    }

    static double frobeniusNorm(double[][] A) {
        double sum = 0.0;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                sum += A[i][j] * A[i][j];
        return Math.sqrt(sum);
    }

    static double vectorNorm(double[] x) {
        return Math.sqrt(x[0] * x[0] + x[1] * x[1]);
    }

    static double[][] inverse2(double[][] A) {
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        return new double[][]{{ A[1][1] / det, -A[0][1] / det},
                              {-A[1][0] / det,  A[0][0] / det}};
    }

    static double[][] matrixExpTaylor2(double[][] A, double t) {
        // Scaling-and-squaring with Taylor series for a small 2x2 teaching example.
        double[][] M = scale(A, t);
        double normM = frobeniusNorm(M);
        int scalingPower = Math.max(0, (int)Math.ceil(Math.log(normM + 1e-12) / Math.log(2.0)));
        double divisor = Math.pow(2.0, scalingPower);
        double[][] B = scale(M, 1.0 / divisor);

        double[][] result = identity2();
        double[][] term = identity2();
        int terms = 40;
        for (int k = 1; k <= terms; k++) {
            term = multiply(term, scale(B, 1.0 / k));
            result = add(result, term);
        }
        for (int i = 0; i < scalingPower; i++) {
            result = multiply(result, result);
        }
        return result;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name + " =");
        for (int i = 0; i < 2; i++) {
            System.out.printf("  [%12.8f, %12.8f]%n", A[i][0], A[i][1]);
        }
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-4.0, -0.8}};
        double t = 0.7;
        double s = 1.3;
        double[] x0 = {1.0, -0.25};

        double[][] Phi_t = matrixExpTaylor2(A, t);
        double[][] Phi_s = matrixExpTaylor2(A, s);
        double[][] Phi_t_plus_s = matrixExpTaylor2(A, t + s);
        double[][] Phi_minus_t = matrixExpTaylor2(A, -t);

        double semigroupError = frobeniusNorm(subtract(Phi_t_plus_s, multiply(Phi_t, Phi_s)));
        double inverseError = frobeniusNorm(subtract(inverse2(Phi_t), Phi_minus_t));
        double identityError = frobeniusNorm(subtract(multiply(Phi_t, Phi_minus_t), identity2()));

        double[] xDirect = multiply(Phi_t_plus_s, x0);
        double[] xTwoStep = multiply(Phi_t, multiply(Phi_s, x0));

        printMatrix("Phi(t)", Phi_t);
        printMatrix("Phi(s)", Phi_s);
        printMatrix("Phi(t+s)", Phi_t_plus_s);
        System.out.println("Semigroup error = " + semigroupError);
        System.out.println("Inverse error = " + inverseError);
        System.out.println("Identity error = " + identityError);
        System.out.println("State two-step propagation error = " + vectorNorm(subtract(xDirect, xTwoStep)));
    }
}
