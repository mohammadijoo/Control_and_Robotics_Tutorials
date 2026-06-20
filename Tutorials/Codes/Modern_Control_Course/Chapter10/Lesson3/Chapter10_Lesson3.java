// Chapter10_Lesson3.java
// Analytic finite-time reachability example for the double integrator.
// Compile and run:
// javac Chapter10_Lesson3.java && java Chapter10_Lesson3

public class Chapter10_Lesson3 {
    static double[][] gramianDoubleIntegrator(double T) {
        return new double[][] {
            {Math.pow(T, 3.0) / 3.0, Math.pow(T, 2.0) / 2.0},
            {Math.pow(T, 2.0) / 2.0, T}
        };
    }

    static double[][] inverse2x2(double[][] M) {
        double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        if (Math.abs(det) < 1e-12) {
            throw new IllegalArgumentException("Matrix is numerically singular.");
        }
        return new double[][] {
            { M[1][1] / det, -M[0][1] / det},
            {-M[1][0] / det,  M[0][0] / det}
        };
    }

    static double[] matVec(double[][] M, double[] v) {
        return new double[] {
            M[0][0] * v[0] + M[0][1] * v[1],
            M[1][0] * v[0] + M[1][1] * v[1]
        };
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    static double uMinEnergy(double T, double t, double[] lambda) {
        // For A=[[0,1],[0,0]], B=[[0],[1]], B' exp(A'(T-t)) = [T-t, 1].
        return (T - t) * lambda[0] + lambda[1];
    }

    public static void main(String[] args) {
        // Steering x0=[0,0] to xT=[1,0] for the double integrator.
        double[] d = {1.0, 0.0};

        for (double T : new double[] {0.5, 1.0, 2.0, 4.0}) {
            double[][] W = gramianDoubleIntegrator(T);
            double[][] Winv = inverse2x2(W);
            double[] lambda = matVec(Winv, d);
            double energy = dot(d, lambda);

            System.out.println("T = " + T);
            System.out.printf("W(T) = [[%.6f, %.6f], [%.6f, %.6f]]%n",
                    W[0][0], W[0][1], W[1][0], W[1][1]);
            System.out.println("minimum energy = " + energy);
            System.out.println("u(0) = " + uMinEnergy(T, 0.0, lambda));
            System.out.println("u(T/2) = " + uMinEnergy(T, T / 2.0, lambda));
            System.out.println("u(T) = " + uMinEnergy(T, T, lambda));
            System.out.println();
        }
    }
}
