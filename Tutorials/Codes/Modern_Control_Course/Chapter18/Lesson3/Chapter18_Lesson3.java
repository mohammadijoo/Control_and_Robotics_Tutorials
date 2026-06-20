// Chapter18_Lesson3.java
// Dynamics associated with a size-3 Jordan block.
//
// Compile:
//   javac Chapter18_Lesson3.java
//
// Run:
//   java Chapter18_Lesson3

public class Chapter18_Lesson3 {
    static double[][] jordanExponential(double lambda, double t) {
        // J = lambda I + N, where N has ones on the first superdiagonal.
        // exp(Jt) = exp(lambda t) * [[1, t, t^2/2], [0, 1, t], [0, 0, 1]].
        double e = Math.exp(lambda * t);
        return new double[][] {
            {e, e * t, e * t * t / 2.0},
            {0.0, e, e * t},
            {0.0, 0.0, e}
        };
    }

    static double[] matVec(double[][] A, double[] x) {
        double[] y = new double[x.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < x.length; j++) {
                y[i] += A[i][j] * x[j];
            }
        }
        return y;
    }

    static void printMatrix(double[][] A) {
        for (double[] row : A) {
            for (double value : row) {
                System.out.printf("%14.7f ", value);
            }
            System.out.println();
        }
    }

    static void printVector(double[] x) {
        System.out.print("[");
        for (int i = 0; i < x.length; i++) {
            System.out.printf("%.7f", x[i]);
            if (i < x.length - 1) {
                System.out.print(", ");
            }
        }
        System.out.print("]");
    }

    public static void main(String[] args) {
        double lambda = -0.40;
        double[] z0 = {1.0, -0.5, 2.0};
        double[] times = {0.0, 1.0, 2.0, 4.0, 8.0};

        System.out.println("Dynamics for zdot = Jz, J = lambda I + N");
        System.out.println("lambda = " + lambda + "\n");

        for (double t : times) {
            double[][] E = jordanExponential(lambda, t);
            double[] z = matVec(E, z0);

            System.out.println("t = " + t);
            System.out.println("exp(Jt) =");
            printMatrix(E);
            System.out.print("z(t) = ");
            printVector(z);
            System.out.println("\n");
        }
    }
}
