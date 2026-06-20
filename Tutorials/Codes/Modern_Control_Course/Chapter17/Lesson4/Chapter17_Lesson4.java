/*
Chapter17_Lesson4.java
Transformations Between Physical and Modal Coordinates

This Java example uses a 2x2 system with known real eigenvectors.
For production numerical work, use a library such as EJML, Apache Commons Math,
Hipparchus, or ojAlgo for eigenvalue computations.
*/

public class Chapter17_Lesson4 {
    static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = new double[A.length][B[0].length];
        for (int i = 0; i &lt; A.length; i++) {
            for (int j = 0; j &lt; B[0].length; j++) {
                for (int k = 0; k &lt; B.length; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[] multiply(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i &lt; A.length; i++) {
            for (int k = 0; k &lt; x.length; k++) {
                y[i] += A[i][k] * x[k];
            }
        }
        return y;
    }

    static double[][] inverse2x2(double[][] A) {
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (Math.abs(det) &lt; 1e-12) {
            throw new IllegalArgumentException("Matrix is singular or nearly singular.");
        }
        return new double[][] {
            { A[1][1] / det, -A[0][1] / det },
            {-A[1][0] / det,  A[0][0] / det }
        };
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name);
        for (double[] row : A) {
            System.out.print("  ");
            for (double v : row) {
                System.out.printf("%10.5f ", v);
            }
            System.out.println();
        }
    }

    static void printVector(String name, double[] x) {
        System.out.printf("%s = [", name);
        for (int i = 0; i &lt; x.length; i++) {
            System.out.printf("%10.5f", x[i]);
            if (i &lt; x.length - 1) System.out.print(", ");
        }
        System.out.println("]^T");
    }

    public static void main(String[] args) {
        double[][] A = { {0.0, 1.0}, {-2.0, -3.0} };
        double[][] T = { {1.0, 1.0}, {-1.0, -2.0} }; // eigenvectors as columns
        double[][] Tinv = inverse2x2(T);
        double[][] Lambda = multiply(multiply(Tinv, A), T);

        double[] B = {0.0, 1.0};
        double[] Bm = multiply(Tinv, B);

        // C = [1, 0], so Cm = C T is the first row of T.
        double[] Cm = {T[0][0], T[0][1]};

        double[] x0 = {2.0, -1.0};
        double[] z0 = multiply(Tinv, x0);
        double[] x0Recovered = multiply(T, z0);

        printMatrix("A:", A);
        printMatrix("T:", T);
        printMatrix("Tinv:", Tinv);
        printMatrix("Lambda = Tinv A T:", Lambda);
        printVector("Bm = Tinv B", Bm);
        printVector("Cm = C T", Cm);
        printVector("z0 = Tinv x0", z0);
        printVector("T z0", x0Recovered);

        System.out.println("\nInterpretation: z1 and z2 are modal amplitudes associated with eigenvalues -1 and -2.");
    }
}
