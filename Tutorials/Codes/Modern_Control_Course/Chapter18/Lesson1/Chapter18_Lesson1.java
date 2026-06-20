// Chapter18_Lesson1.java
// Scratch verification of generalized eigenvectors for a Jordan chain.
// Compile:
//   javac Chapter18_Lesson1.java
// Run:
//   java Chapter18_Lesson1

public class Chapter18_Lesson1 {
    static double[] matVec(double[][] A, double[] x) {
        int n = A.length;
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            y[i] = 0.0;
            for (int j = 0; j < x.length; j++) {
                y[i] += A[i][j] * x[j];
            }
        }
        return y;
    }

    static double[][] subtractLambdaI(double[][] A, double lambda) {
        int n = A.length;
        double[][] N = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                N[i][j] = A[i][j];
            }
            N[i][i] -= lambda;
        }
        return N;
    }

    static double norm2(double[] x) {
        double s = 0.0;
        for (double xi : x) {
            s += xi * xi;
        }
        return Math.sqrt(s);
    }

    static double[] subtract(double[] a, double[] b) {
        double[] c = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            c[i] = a[i] - b[i];
        }
        return c;
    }

    static void printVector(String name, double[] x) {
        System.out.print(name + " = [");
        for (int i = 0; i < x.length; i++) {
            System.out.printf("%8.4f", x[i]);
            if (i + 1 < x.length) {
                System.out.print(", ");
            }
        }
        System.out.println("]^T");
    }

    public static void main(String[] args) {
        double[][] A = {
            {2, 1, 0, 0},
            {0, 2, 1, 0},
            {0, 0, 2, 0},
            {0, 0, 0, -1}
        };

        double lambda = 2.0;
        double[][] N = subtractLambdaI(A, lambda);

        double[] v3 = {0, 0, 1, 0};
        double[] v2 = matVec(N, v3);
        double[] v1 = matVec(N, v2);

        System.out.println("Jordan chain for lambda = 2");
        printVector("v1", v1);
        printVector("v2", v2);
        printVector("v3", v3);

        double[] Nv1 = matVec(N, v1);
        double[] Nv2 = matVec(N, v2);
        double[] Nv3 = matVec(N, v3);

        System.out.println("\nVerification:");
        printVector("N*v1", Nv1);
        printVector("N*v2", Nv2);
        printVector("N*v3", Nv3);

        System.out.println("\nResidual norms:");
        System.out.printf("||N*v1||_2      = %.6e%n", norm2(Nv1));
        System.out.printf("||N*v2 - v1||_2 = %.6e%n", norm2(subtract(Nv2, v1)));
        System.out.printf("||N*v3 - v2||_2 = %.6e%n", norm2(subtract(Nv3, v2)));
    }
}
