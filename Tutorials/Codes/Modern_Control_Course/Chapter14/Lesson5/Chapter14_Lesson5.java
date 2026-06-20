// Chapter14_Lesson5.java
// Continuous-time LTV observability Gramian using RK4 for a 2-state example.

public class Chapter14_Lesson5 {
    static double[][] zeros() {
        return new double[][] {{0.0, 0.0}, {0.0, 0.0}};
    }

    static double[][] identity() {
        return new double[][] {{1.0, 0.0}, {0.0, 1.0}};
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                C[i][j] = s * A[i][j];
        return C;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] transpose(double[][] A) {
        return new double[][] {{A[0][0], A[1][0]}, {A[0][1], A[1][1]}};
    }

    static double[][] AofT(double t) {
        return new double[][] {
            {0.0, 1.0},
            {-(2.0 + 0.5 * Math.sin(1.3 * t)), -(0.15 + 0.05 * Math.cos(t))}
        };
    }

    static double[][] CtCofT(double t) {
        double c1 = 1.0;
        double c2 = 0.2 * Math.sin(0.7 * t);
        return new double[][] {{c1 * c1, c1 * c2}, {c2 * c1, c2 * c2}};
    }

    static double[][] phiRhs(double t, double[][] Phi) {
        return multiply(AofT(t), Phi);
    }

    static double[][] rk4PhiStep(double t, double[][] Phi, double h) {
        double[][] k1 = phiRhs(t, Phi);
        double[][] k2 = phiRhs(t + 0.5 * h, add(Phi, scale(k1, 0.5 * h)));
        double[][] k3 = phiRhs(t + 0.5 * h, add(Phi, scale(k2, 0.5 * h)));
        double[][] k4 = phiRhs(t + h, add(Phi, scale(k3, h)));

        double[][] incr = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
        return add(Phi, scale(incr, h / 6.0));
    }

    static double[][] gramianIntegrand(double t, double[][] Phi) {
        return multiply(multiply(transpose(Phi), CtCofT(t)), Phi);
    }

    static void printMatrix(double[][] M) {
        for (int i = 0; i < 2; i++) {
            System.out.printf("[ %12.8f %12.8f ]%n", M[i][0], M[i][1]);
        }
    }

    public static void main(String[] args) {
        double t0 = 0.0;
        double tf = 6.0;
        int steps = 6000;
        double h = (tf - t0) / steps;

        double[][] Phi = identity();
        double[][] W = zeros();

        double t = t0;
        double[][] Fprev = gramianIntegrand(t, Phi);

        for (int k = 0; k < steps; k++) {
            double[][] PhiNext = rk4PhiStep(t, Phi, h);
            double tNext = t + h;
            double[][] Fnext = gramianIntegrand(tNext, PhiNext);

            W = add(W, scale(add(Fprev, Fnext), 0.5 * h));

            Phi = PhiNext;
            Fprev = Fnext;
            t = tNext;
        }

        double detW = W[0][0] * W[1][1] - W[0][1] * W[1][0];

        System.out.println("Approximate observability Gramian W_o:");
        printMatrix(W);
        System.out.printf("det(W_o) = %.12f%n", detW);
        if (detW > 1e-8) {
            System.out.println("The numerical test indicates observability on [0, 6].");
        } else {
            System.out.println("The numerical test indicates loss or near-loss of observability.");
        }
    }
}
