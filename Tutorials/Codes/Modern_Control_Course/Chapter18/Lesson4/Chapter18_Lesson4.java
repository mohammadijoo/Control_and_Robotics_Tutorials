/*
Chapter18_Lesson4.java
Repeated eigenvalues and non-diagonalizable systems.

Self-contained computation for a 3 x 3 Jordan block.
Compile:
    javac Chapter18_Lesson4.java
Run:
    java Chapter18_Lesson4
*/

public class Chapter18_Lesson4 {
    static double[][] expJordan3(double lambda, double t) {
        double e = Math.exp(lambda * t);
        return new double[][] {
            {e, e * t, e * t * t / 2.0},
            {0.0, e, e * t},
            {0.0, 0.0, e}
        };
    }

    static double[] multiply(double[][] a, double[] x) {
        double[] y = new double[3];
        for (int i = 0; i < 3; i++) {
            y[i] = 0.0;
            for (int j = 0; j < 3; j++) {
                y[i] += a[i][j] * x[j];
            }
        }
        return y;
    }

    public static void main(String[] args) {
        double lambda = -0.4;
        double[] x0 = {1.0, -2.0, 1.5};

        System.out.println("Chapter 18 Lesson 4: Jordan response for a defective repeated eigenvalue");
        System.out.println("A = [[lambda,1,0],[0,lambda,1],[0,0,lambda]], lambda = " + lambda);
        System.out.println();
        System.out.printf("%6s%14s%14s%14s%n", "t", "x1(t)", "x2(t)", "x3(t)");

        for (int k = 0; k <= 8; k++) {
            double t = (double) k;
            double[][] phi = expJordan3(lambda, t);
            double[] x = multiply(phi, x0);
            System.out.printf("%6.1f%14.6f%14.6f%14.6f%n", t, x[0], x[1], x[2]);
        }
    }
}
