// Chapter12_Lesson3.java
// Finite-horizon Gramian test for 2-state continuous-time LTI systems.
// Uses only the Java standard library.

public class Chapter12_Lesson3 {
    static double[][] zero2() {
        return new double[][]{{0.0, 0.0}, {0.0, 0.0}};
    }

    static double[][] identity2() {
        return new double[][]{{1.0, 0.0}, {0.0, 1.0}};
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

    static double[][] multiply(double[][] X, double[][] Y) {
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

    static double[][] expmTaylor(double[][] A, double t, int terms) {
        double[][] At = scale(A, t);
        double[][] E = identity2();
        double[][] P = identity2();
        double factorial = 1.0;

        for (int k = 1; k <= terms; k++) {
            P = multiply(P, At);
            factorial *= k;
            E = add(E, scale(P, 1.0 / factorial));
        }
        return E;
    }

    static double[][] integrand(double[][] A, double[] b, double tau) {
        double[][] E = expmTaylor(A, tau, 40);
        double[][] BBt = outer(b);
        return multiply(multiply(E, BBt), transpose(E));
    }

    static double[][] finiteHorizonGramian(double[][] A, double[] b, double T, int N) {
        if (N % 2 == 1) N++;
        double h = T / N;
        double[][] W = zero2();

        for (int k = 0; k <= N; k++) {
            double tau = k * h;
            double coeff = (k == 0 || k == N) ? 1.0 : ((k % 2 == 0) ? 2.0 : 4.0);
            W = add(W, scale(integrand(A, b, tau), coeff));
        }

        W = scale(W, h / 3.0);
        double off = 0.5 * (W[0][1] + W[1][0]);
        W[0][1] = off;
        W[1][0] = off;
        return W;
    }

    static double det2(double[][] W) {
        return W[0][0] * W[1][1] - W[0][1] * W[1][0];
    }

    static double[] symmetricEigenvalues(double[][] W) {
        double a = W[0][0];
        double d = W[1][1];
        double c = 0.5 * (W[0][1] + W[1][0]);
        double tr = a + d;
        double disc = Math.sqrt((a - d) * (a - d) + 4.0 * c * c);
        return new double[]{0.5 * (tr - disc), 0.5 * (tr + disc)};
    }

    static void printMatrix(double[][] W) {
        System.out.printf("[%.10f, %.10f]%n", W[0][0], W[0][1]);
        System.out.printf("[%.10f, %.10f]%n", W[1][0], W[1][1]);
    }

    static void report(double[][] W) {
        double[] ev = symmetricEigenvalues(W);
        double tol = 1e-8;
        System.out.println("det(W) = " + det2(W));
        System.out.println("lambda_min = " + ev[0]);
        System.out.println("lambda_max = " + ev[1]);
        System.out.println("positive definite? " + (ev[0] > tol ? "yes" : "no"));
        if (ev[0] > tol)
            System.out.println("condition number = " + (ev[1] / ev[0]));
        else
            System.out.println("condition number = infinity");
    }

    public static void main(String[] args) {
        double[][] A1 = {{0.0, 1.0}, {-2.0, -3.0}};
        double[] b1 = {0.0, 1.0};
        double T = 2.0;

        System.out.println("Controllable example Wc(T):");
        double[][] W1 = finiteHorizonGramian(A1, b1, T, 2000);
        printMatrix(W1);
        report(W1);

        double[][] A2 = {{0.0, 0.0}, {0.0, -1.0}};
        double[] b2 = {1.0, 0.0};

        System.out.println("\nUncontrollable example Wc(T):");
        double[][] W2 = finiteHorizonGramian(A2, b2, T, 2000);
        printMatrix(W2);
        report(W2);
    }
}
