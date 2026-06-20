// Chapter30_Lesson4.java
// Integrated case study: mass-spring-damper model to state-feedback controller
// This implementation uses only core Java. For larger systems, use Apache Commons Math or EJML.

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter30_Lesson4 {
    static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1]};
    }

    static double[] scale(double c, double[] a) {
        return new double[] {c * a[0], c * a[1]};
    }

    static double[] matVec(double[][] A, double[] x) {
        return new double[] {
            A[0][0] * x[0] + A[0][1] * x[1],
            A[1][0] * x[0] + A[1][1] * x[1]
        };
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    static double[] dynamics(double[][] Acl, double[] Bcl, double reference, double[] x) {
        double[] ax = matVec(Acl, x);
        return add(ax, scale(reference, Bcl));
    }

    static double[] rk4Step(double[][] Acl, double[] Bcl, double reference, double[] x, double h) {
        double[] k1 = dynamics(Acl, Bcl, reference, x);
        double[] k2 = dynamics(Acl, Bcl, reference, add(x, scale(0.5 * h, k1)));
        double[] k3 = dynamics(Acl, Bcl, reference, add(x, scale(0.5 * h, k2)));
        double[] k4 = dynamics(Acl, Bcl, reference, add(x, scale(h, k3)));
        return add(x, scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4))));
    }

    public static void main(String[] args) throws IOException {
        double m = 1.2;
        double b = 0.8;
        double k = 3.0;
        double zeta = 0.70;
        double settlingTime = 2.0;
        double omegaN = 4.0 / (zeta * settlingTime);

        double[][] A = {{0.0, 1.0}, {-k / m, -b / m}};
        double[] B = {0.0, 1.0 / m};
        double[] C = {1.0, 0.0};

        double k1 = m * omegaN * omegaN - k;
        double k2 = 2.0 * zeta * omegaN * m - b;
        double[] K = {k1, k2};
        double nbar = k + k1;

        double[][] Acl = {{0.0, 1.0}, {-(k + k1) / m, -(b + k2) / m}};
        double[] Bcl = {0.0, nbar / m};

        double trace = Acl[0][0] + Acl[1][1];
        double determinant = Acl[0][0] * Acl[1][1] - Acl[0][1] * Acl[1][0];

        System.out.printf("K = [%.8f, %.8f]%n", K[0], K[1]);
        System.out.printf("Nbar = %.8f%n", nbar);
        System.out.printf("Closed-loop characteristic: s^2 - (%.8f) s + %.8f%n", trace, determinant);

        double reference = 1.0;
        double dt = 0.002;
        double tFinal = 6.0;
        int steps = (int) (tFinal / dt);
        double[] x = {0.0, 0.0};
        double maxAbsU = 0.0;

        try (PrintWriter csv = new PrintWriter(new FileWriter("Chapter30_Lesson4_response_java.csv"))) {
            csv.println("t,position,velocity,output,control");
            for (int i = 0; i <= steps; i++) {
                double t = i * dt;
                double y = dot(C, x);
                double u = -dot(K, x) + nbar * reference;
                maxAbsU = Math.max(maxAbsU, Math.abs(u));
                csv.printf("%.6f,%.10f,%.10f,%.10f,%.10f%n", t, x[0], x[1], y, u);
                if (i < steps) x = rk4Step(Acl, Bcl, reference, x, dt);
            }
        }

        System.out.printf("Final output = %.8f%n", dot(C, x));
        System.out.printf("Maximum absolute control input = %.8f%n", maxAbsU);
    }
}
