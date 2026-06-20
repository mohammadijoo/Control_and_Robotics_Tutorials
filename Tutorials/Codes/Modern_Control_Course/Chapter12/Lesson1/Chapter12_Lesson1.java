// Chapter12_Lesson1.java
// Finite-horizon controllability Gramian for a 2 by 2 example.
// Compile and run:
//   javac Chapter12_Lesson1.java
//   java Chapter12_Lesson1
//
// The code integrates:
//   dW/dt = A W + W A^T + B B^T,   W(0) = 0
// using fourth-order Runge-Kutta.

public class Chapter12_Lesson1 {
    static double[][] zero2() {
        return new double[][]{{0.0, 0.0}, {0.0, 0.0}};
    }

    static double[][] add(double[][] X, double[][] Y) {
        double[][] Z = zero2();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                Z[i][j] = X[i][j] + Y[i][j];
        return Z;
    }

    static double[][] scale(double[][] X, double a) {
        double[][] Z = zero2();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                Z[i][j] = a * X[i][j];
        return Z;
    }

    static double[][] mul(double[][] X, double[][] Y) {
        double[][] Z = zero2();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    Z[i][j] += X[i][k] * Y[k][j];
        return Z;
    }

    static double[][] transpose(double[][] X) {
        return new double[][]{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}};
    }

    static double[][] outer(double[] b) {
        return new double[][]{
            {b[0] * b[0], b[0] * b[1]},
            {b[1] * b[0], b[1] * b[1]}
        };
    }

    static double[][] rhs(double[][] A, double[] b, double[][] W) {
        double[][] AT = transpose(A);
        return add(add(mul(A, W), mul(W, AT)), outer(b));
    }

    static double[][] rk4Step(double[][] A, double[] b, double[][] W, double h) {
        double[][] k1 = rhs(A, b, W);
        double[][] k2 = rhs(A, b, add(W, scale(k1, 0.5 * h)));
        double[][] k3 = rhs(A, b, add(W, scale(k2, 0.5 * h)));
        double[][] k4 = rhs(A, b, add(W, scale(k3, h)));

        double[][] update = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
        return add(W, scale(update, h / 6.0));
    }

    static double[][] finiteHorizonGramian(double[][] A, double[] b, double T, int steps) {
        double[][] W = zero2();
        double h = T / steps;
        for (int k = 0; k < steps; k++) {
            W = rk4Step(A, b, W, h);
        }
        double off = 0.5 * (W[0][1] + W[1][0]);
        W[0][1] = off;
        W[1][0] = off;
        return W;
    }

    static double det2(double[][] X) {
        return X[0][0] * X[1][1] - X[0][1] * X[1][0];
    }

    static void printMatrix(String name, double[][] X) {
        System.out.println("\n" + name + " =");
        for (int i = 0; i < 2; i++) {
            System.out.printf("[ %12.7f %12.7f ]%n", X[i][0], X[i][1]);
        }
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-2.0, -3.0}};
        double[] b = {0.0, 1.0};

        double T = 2.0;
        int steps = 20000;

        double[][] W = finiteHorizonGramian(A, b, T, steps);
        printMatrix("Wc(T)", W);

        double determinant = det2(W);
        System.out.printf("%ndet(Wc(T)) = %.12f%n", determinant);

        if (determinant > 1e-10) {
            System.out.println("The Gramian is nonsingular: the system is controllable on this horizon.");
        } else {
            System.out.println("The Gramian is singular or nearly singular.");
        }
    }
}
