/*
Chapter27_Lesson5.java

Reference tracking and disturbance rejection case study in Java.

Compile:
    javac Chapter27_Lesson5.java
Run:
    java Chapter27_Lesson5
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter27_Lesson5 {
    static class Params {
        double m = 1.0;
        double b = 0.6;
        double k = 2.0;
        double r = 1.0;
        double d = 0.4;
    }

    static double[] derivFeedforward(double[] x, Params p, double K1, double K2, double nbar, double d) {
        double u = -K1 * x[0] - K2 * x[1] + nbar * p.r;
        return new double[] {
            x[1],
            (-p.k * x[0] - p.b * x[1] + u + d) / p.m
        };
    }

    static double[] derivIntegral(double[] x, Params p, double K1, double K2, double Ki, double d) {
        double u = -K1 * x[0] - K2 * x[1] + Ki * x[2];
        return new double[] {
            x[1],
            (-p.k * x[0] - p.b * x[1] + u + d) / p.m,
            p.r - x[0]
        };
    }

    static double[] addScaled(double[] x, double[] k, double scale) {
        double[] y = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            y[i] = x[i] + scale * k[i];
        }
        return y;
    }

    static double[] rk4(double[] x, double h, Derivative f) {
        double[] k1 = f.eval(x);
        double[] k2 = f.eval(addScaled(x, k1, 0.5 * h));
        double[] k3 = f.eval(addScaled(x, k2, 0.5 * h));
        double[] k4 = f.eval(addScaled(x, k3, h));

        double[] y = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            y[i] = x[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
        }
        return y;
    }

    interface Derivative {
        double[] eval(double[] x);
    }

    public static void main(String[] args) throws IOException {
        Params p = new Params();

        // Feedforward design: desired polynomial (s+2)(s+3)=s^2+5s+6.
        double a1 = 5.0;
        double a0 = 6.0;
        double K1 = p.m * a0 - p.k;
        double K2 = p.m * a1 - p.b;
        double nbar = p.k + K1;

        // Integral design: desired polynomial (s+2)(s+3)(s+5)=s^3+10s^2+31s+30.
        double alpha2 = 10.0;
        double alpha1 = 31.0;
        double alpha0 = 30.0;
        double K1i = p.m * alpha1 - p.k;
        double K2i = p.m * alpha2 - p.b;
        double Ki = p.m * alpha0;

        double h = 0.001;
        double tf = 8.0;
        int steps = (int)(tf / h);

        double[] xff0 = {0.0, 0.0};
        double[] xff1 = {0.0, 0.0};
        double[] xi = {0.0, 0.0, 0.0};

        try (PrintWriter out = new PrintWriter(new FileWriter("Chapter27_Lesson5_java_results.csv"))) {
            out.println("t,y_feedforward_no_dist,y_feedforward_with_dist,y_integral_with_dist,u_integral");

            for (int i = 0; i <= steps; i++) {
                double t = i * h;
                double ui = -K1i * xi[0] - K2i * xi[1] + Ki * xi[2];

                if (i % 10 == 0) {
                    out.printf("%.6f,%.10f,%.10f,%.10f,%.10f%n", t, xff0[0], xff1[0], xi[0], ui);
                }

                xff0 = rk4(xff0, h, x -> derivFeedforward(x, p, K1, K2, nbar, 0.0));
                xff1 = rk4(xff1, h, x -> derivFeedforward(x, p, K1, K2, nbar, p.d));
                xi = rk4(xi, h, x -> derivIntegral(x, p, K1i, K2i, Ki, p.d));
            }
        }

        System.out.println("Feedforward gains: K1=" + K1 + ", K2=" + K2 + ", Nbar=" + nbar);
        System.out.println("Integral gains: K1=" + K1i + ", K2=" + K2i + ", Ki=" + Ki);
        System.out.println("Final feedforward y without disturbance: " + xff0[0]);
        System.out.println("Final feedforward y with disturbance: " + xff1[0]);
        System.out.println("Final integral y with disturbance: " + xi[0]);
        System.out.println("CSV written to Chapter27_Lesson5_java_results.csv");
    }
}
