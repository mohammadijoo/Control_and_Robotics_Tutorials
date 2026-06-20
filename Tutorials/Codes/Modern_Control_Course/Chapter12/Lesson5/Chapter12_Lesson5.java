// Chapter12_Lesson5.java
// Gramian and Hankel-singular-value preview without external libraries.
// Compile: javac Chapter12_Lesson5.java
// Run:     java Chapter12_Lesson5

public class Chapter12_Lesson5 {
    static double[][] transpose(double[][] X) {
        return new double[][]{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}};
    }

    static double[][] add(double[][] X, double[][] Y) {
        double[][] Z = new double[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                Z[i][j] = X[i][j] + Y[i][j];
        return Z;
    }

    static double[][] mul(double[][] X, double[][] Y) {
        double[][] Z = new double[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    Z[i][j] += X[i][k] * Y[k][j];
        return Z;
    }

    static double[] flatten(double[][] X) {
        return new double[]{X[0][0], X[0][1], X[1][0], X[1][1]};
    }

    static double[][] basis(int index) {
        double[][] E = new double[2][2];
        E[index / 2][index % 2] = 1.0;
        return E;
    }

    static double[] solve4(double[][] A, double[] b) {
        int n = 4;
        for (int k = 0; k < n; k++) {
            int pivot = k;
            for (int i = k + 1; i < n; i++)
                if (Math.abs(A[i][k]) > Math.abs(A[pivot][k])) pivot = i;

            if (Math.abs(A[pivot][k]) < 1e-12)
                throw new RuntimeException("Singular system");

            double[] tempRow = A[k]; A[k] = A[pivot]; A[pivot] = tempRow;
            double tempVal = b[k]; b[k] = b[pivot]; b[pivot] = tempVal;

            double diag = A[k][k];
            for (int j = k; j < n; j++) A[k][j] /= diag;
            b[k] /= diag;

            for (int i = 0; i < n; i++) {
                if (i == k) continue;
                double factor = A[i][k];
                for (int j = k; j < n; j++) A[i][j] -= factor * A[k][j];
                b[i] -= factor * b[k];
            }
        }
        return b;
    }

    // Solve A W + W A^T + Q = 0.
    static double[][] lyapunovSolve(double[][] A, double[][] Q) {
        double[][] K = new double[4][4];
        for (int col = 0; col < 4; col++) {
            double[][] E = basis(col);
            double[][] L = add(mul(A, E), mul(E, transpose(A)));
            double[] f = flatten(L);
            for (int row = 0; row < 4; row++) K[row][col] = f[row];
        }

        double[] rhs = flatten(Q);
        for (int i = 0; i < rhs.length; i++) rhs[i] = -rhs[i];

        double[] sol = solve4(K, rhs);
        return new double[][]{{sol[0], sol[1]}, {sol[2], sol[3]}};
    }

    static double[] eigenvalues2(double[][] X) {
        double tr = X[0][0] + X[1][1];
        double det = X[0][0] * X[1][1] - X[0][1] * X[1][0];
        double disc = Math.max(0.0, tr * tr - 4.0 * det);
        double r1 = 0.5 * (tr + Math.sqrt(disc));
        double r2 = 0.5 * (tr - Math.sqrt(disc));
        return (r1 >= r2) ? new double[]{r1, r2} : new double[]{r2, r1};
    }

    static void printMat(String name, double[][] X) {
        System.out.println(name);
        for (int i = 0; i < 2; i++)
            System.out.printf("  %12.6f %12.6f%n", X[i][0], X[i][1]);
    }

    public static void main(String[] args) {
        double[][] A = {{-1.0, 0.3}, {0.0, -2.0}};

        // B = [1.0; 0.5], so B B^T:
        double[][] BBt = {{1.0, 0.5}, {0.5, 0.25}};

        // C = [1.0, -0.2], so C^T C:
        double[][] CtC = {{1.0, -0.2}, {-0.2, 0.04}};

        double[][] Wc = lyapunovSolve(A, BBt);
        double[][] Wo = lyapunovSolve(transpose(A), CtC);
        double[][] product = mul(Wc, Wo);
        double[] lambda = eigenvalues2(product);

        printMat("Controllability Gramian Wc:", Wc);
        printMat("\nOutput-energy Gramian Wo:", Wo);
        printMat("\nProduct Wc*Wo:", product);

        System.out.println("\nHankel singular values:");
        System.out.printf("  sigma1 = %.6f%n", Math.sqrt(Math.max(0.0, lambda[0])));
        System.out.printf("  sigma2 = %.6f%n", Math.sqrt(Math.max(0.0, lambda[1])));

        System.out.println("\nInterpretation: large sigma means a state direction is both reachable with small input energy and visible at the output.");
    }
}
