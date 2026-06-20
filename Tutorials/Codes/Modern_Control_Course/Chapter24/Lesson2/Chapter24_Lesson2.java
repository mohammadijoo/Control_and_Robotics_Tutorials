// Chapter24_Lesson2.java
// Degrees of freedom and closed-loop eigenstructure for MIMO pole placement.
// No external library is required for this small 3-state example.
//
// Compile:
//   javac Chapter24_Lesson2.java
//
// Run:
//   java Chapter24_Lesson2

public class Chapter24_Lesson2 {

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double x : row) {
                System.out.printf("%14.8f ", x);
            }
            System.out.println();
        }
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int c = B[0].length;
        int inner = B.length;
        double[][] C = new double[r][c];

        for (int i = 0; i < r; i++) {
            for (int k = 0; k < inner; k++) {
                for (int j = 0; j < c; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double det3(double[][] M) {
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
             - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
             + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    }

    static double[][] inverse3(double[][] M) {
        double d = det3(M);
        if (Math.abs(d) < 1e-12) {
            throw new IllegalArgumentException("Singular or nearly singular V matrix.");
        }

        double[][] Inv = new double[3][3];

        Inv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / d;
        Inv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) / d;
        Inv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / d;

        Inv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) / d;
        Inv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / d;
        Inv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) / d;

        Inv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / d;
        Inv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) / d;
        Inv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / d;

        return Inv;
    }

    static double[] eigenvector(double lambda, double gamma) {
        return new double[] {1.0, lambda, lambda * lambda + gamma};
    }

    static double[] zColumn(double lambda, double gamma) {
        double v3 = lambda * lambda + gamma;
        double z1 = gamma;
        double z2 = -2.0 - 3.0 * lambda + (-4.0 - lambda) * v3;
        return new double[] {z1, z2};
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {-2.0, -3.0, -4.0}
        };

        double[][] B = {
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 1.0}
        };

        double[] lambdas = {-1.0, -2.0, -5.0};
        double[] gammas  = {0.2, -0.4, 0.8};

        double[][] V = new double[3][3];
        double[][] Z = new double[2][3];

        for (int j = 0; j < 3; j++) {
            double[] v = eigenvector(lambdas[j], gammas[j]);
            double[] z = zColumn(lambdas[j], gammas[j]);

            for (int i = 0; i < 3; i++) V[i][j] = v[i];
            for (int i = 0; i < 2; i++) Z[i][j] = z[i];
        }

        double[][] K = multiply(Z, inverse3(V));

        printMatrix("A", A);
        printMatrix("B", B);
        printMatrix("V", V);
        printMatrix("Z = K V", Z);
        printMatrix("K", K);

        System.out.print("\nThe desired eigenvalues are: ");
        for (double x : lambdas) System.out.print(x + " ");
        System.out.println("\nFor this construction, A V - B Z = V Lambda by design.");
        System.out.println("Closed-loop matrix is A - B K.");
    }
}
