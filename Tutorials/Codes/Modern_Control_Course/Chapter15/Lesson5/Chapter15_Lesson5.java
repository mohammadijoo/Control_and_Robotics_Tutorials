// Chapter15_Lesson5.java
// From-scratch finite-horizon Gramian integration for a 2x2 example.
// Compile:
//     javac Chapter15_Lesson5.java
// Run:
//     java Chapter15_Lesson5

public class Chapter15_Lesson5 {
    static double[][] zeros() {
        return new double[][] {{0.0, 0.0}, {0.0, 0.0}};
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

    static double[][] mul(double[][] A, double[][] B) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] trans(double[][] A) {
        return new double[][] {{A[0][0], A[1][0]}, {A[0][1], A[1][1]}};
    }

    static double frobeniusNorm(double[][] A) {
        double s = 0.0;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                s += A[i][j] * A[i][j];
        return Math.sqrt(s);
    }

    static double[][] expm2x2(double[][] A, double t) {
        // Scaling-and-squaring with truncated Taylor series, sufficient for this demonstration.
        int squarings = 6;
        double factor = Math.pow(2.0, squarings);
        double[][] X = scale(A, t / factor);

        double[][] E = {{1.0, 0.0}, {0.0, 1.0}};
        double[][] term = {{1.0, 0.0}, {0.0, 1.0}};

        for (int k = 1; k <= 24; k++) {
            term = scale(mul(term, X), 1.0 / k);
            E = add(E, term);
        }

        for (int i = 0; i < squarings; i++) {
            E = mul(E, E);
        }
        return E;
    }

    static double[][] observabilityIntegrand(double[][] A, double[] C, double t) {
        double[][] E = expm2x2(A, t);
        double[][] Q = {{C[0] * C[0], C[0] * C[1]},
                        {C[1] * C[0], C[1] * C[1]}};
        return mul(mul(trans(E), Q), E);
    }

    static double[][] controllabilityIntegrand(double[][] A, double[] B, double t) {
        double[][] E = expm2x2(A, t);
        double[][] R = {{B[0] * B[0], B[0] * B[1]},
                        {B[1] * B[0], B[1] * B[1]}};
        return mul(mul(E, R), trans(E));
    }

    static double[][] integrateObservability(double[][] A, double[] C, double T, int steps) {
        double[][] W = zeros();
        double dt = T / steps;
        for (int k = 0; k <= steps; k++) {
            double weight = (k == 0 || k == steps) ? 0.5 : 1.0;
            W = add(W, scale(observabilityIntegrand(A, C, k * dt), weight));
        }
        return scale(W, dt);
    }

    static double[][] integrateControllability(double[][] A, double[] B, double T, int steps) {
        double[][] W = zeros();
        double dt = T / steps;
        for (int k = 0; k <= steps; k++) {
            double weight = (k == 0 || k == steps) ? 0.5 : 1.0;
            W = add(W, scale(controllabilityIntegrand(A, B, k * dt), weight));
        }
        return scale(W, dt);
    }

    static void printMatrix(double[][] A) {
        for (int i = 0; i < 2; i++) {
            System.out.printf("[ %.8f  %.8f ]%n", A[i][0], A[i][1]);
        }
    }

    public static void main(String[] args) {
        double[][] A = {{-1.0, 2.0}, {-3.0, -4.0}};
        double[][] AT = trans(A);
        double[] C = {1.0, 0.5};
        double[] Bdual = {1.0, 0.5};

        double T = 3.0;
        int steps = 4000;

        double[][] Wo = integrateObservability(A, C, T, steps);
        double[][] WcDual = integrateControllability(AT, Bdual, T, steps);

        System.out.println("Finite-horizon observability Gramian W_o(A,C):");
        printMatrix(Wo);

        System.out.println("\nFinite-horizon controllability Gramian W_c(A^T,C^T):");
        printMatrix(WcDual);

        double[][] D = add(Wo, scale(WcDual, -1.0));
        System.out.println("\nFrobenius difference: " + frobeniusNorm(D));
    }
}
