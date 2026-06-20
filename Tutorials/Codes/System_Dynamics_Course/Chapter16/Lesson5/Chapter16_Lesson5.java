// Chapter16_Lesson5.java
// Discrete-time stability and step response demo in Java (no external libraries)
import java.util.Arrays;

public class Chapter16_Lesson5 {

    static class Complex {
        double re, im;
        Complex(double re, double im) { this.re = re; this.im = im; }
        double abs() { return Math.hypot(re, im); }
        public String toString() {
            if (Math.abs(im) < 1e-12) return String.format("%.6f", re);
            return String.format("%.6f %s j%.6f", re, (im >= 0 ? "+" : "-"), Math.abs(im));
        }
    }

    static double trace2(double[][] A) { return A[0][0] + A[1][1]; }
    static double det2(double[][] A) { return A[0][0]*A[1][1] - A[0][1]*A[1][0]; }

    static Complex[] eig2x2(double[][] A) {
        double tr = trace2(A);
        double det = det2(A);
        double disc = tr*tr - 4.0*det;
        if (disc >= 0) {
            double s = Math.sqrt(disc);
            return new Complex[] {
                new Complex(0.5*(tr + s), 0.0),
                new Complex(0.5*(tr - s), 0.0)
            };
        } else {
            double re = 0.5*tr;
            double im = 0.5*Math.sqrt(-disc);
            return new Complex[] {
                new Complex(re, im),
                new Complex(re, -im)
            };
        }
    }

    // Jury criterion specialized to second-order polynomial z^2 + a1 z + a0
    static boolean jurySecondOrder(double a1, double a0) {
        return (Math.abs(a0) < 1.0) && (1.0 + a1 + a0 > 0.0) && (1.0 - a1 + a0 > 0.0);
    }

    public static void main(String[] args) {
        double[][] A = {
            {1.5770, -0.6724},
            {1.0000,  0.0000}
        };
        double[] B = {1.0, 0.0};
        double[] C = {0.0676, 0.0604};
        double D = 0.0;

        double a1 = -trace2(A);
        double a0 = det2(A);

        Complex[] eigs = eig2x2(A);
        System.out.printf("Characteristic polynomial: z^2 + (%.6f) z + (%.6f)%n", a1, a0);
        System.out.println("Jury stable? " + jurySecondOrder(a1, a0));
        System.out.println("Eigenvalues: " + eigs[0] + ", " + eigs[1]);
        System.out.printf("Magnitudes: %.6f, %.6f%n", eigs[0].abs(), eigs[1].abs());

        int N = 70;
        double[] x = {0.0, 0.0};
        double[] y = new double[N];

        for (int k = 0; k < N; k++) {
            double u = 1.0;
            y[k] = C[0]*x[0] + C[1]*x[1] + D*u;

            double[] xn = new double[2];
            xn[0] = A[0][0]*x[0] + A[0][1]*x[1] + B[0]*u;
            xn[1] = A[1][0]*x[0] + A[1][1]*x[1] + B[1]*u;
            x = xn;
        }

        System.out.println("\nFirst 12 step samples:");
        for (int k = 0; k < 12; k++) {
            System.out.printf("k=%2d  y=%.6f%n", k, y[k]);
        }

        // 2% settling index about final value 1
        int settle = -1;
        for (int k = 0; k < N; k++) {
            boolean ok = true;
            for (int j = k; j < N; j++) {
                if (Math.abs(y[j] - 1.0) > 0.02) { ok = false; break; }
            }
            if (ok) { settle = k; break; }
        }
        System.out.println("\n2% settling sample index = " + settle);

        // Optional: natural response from nonzero initial condition
        x = new double[]{1.0, -0.4};
        double[] yn = new double[12];
        for (int k = 0; k < 12; k++) {
            yn[k] = C[0]*x[0] + C[1]*x[1];
            double[] xn = new double[2];
            xn[0] = A[0][0]*x[0] + A[0][1]*x[1];
            xn[1] = A[1][0]*x[0] + A[1][1]*x[1];
            x = xn;
        }
        System.out.println("\nNatural response first 12 samples:");
        System.out.println(Arrays.toString(yn));
    }
}
