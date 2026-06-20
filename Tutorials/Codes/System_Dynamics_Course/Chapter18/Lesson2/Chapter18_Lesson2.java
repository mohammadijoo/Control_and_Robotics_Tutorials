// Chapter18_Lesson2.java
// Lagrangian cart-pendulum simulation (generalized coordinates q = [x, theta])
// Pure Java implementation with RK4 integration and CSV export.

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter18_Lesson2 {

    static class Params {
        double M = 1.0;
        double m = 0.25;
        double l = 0.5;
        double J = 0.02;
        double g = 9.81;
        double k = 8.0;
        double c = 0.35;
        double b = 0.05;
    }

    static double controlInput(double t) {
        return (t >= 0.5 && t <= 2.5) ? 2.0 : 0.0;
    }

    static double[] rhs(double t, double[] s, Params p) {
        double x = s[0];
        double th = s[1];
        double xd = s[2];
        double thd = s[3];

        double M11 = p.M + p.m;
        double M12 = p.m * p.l * Math.cos(th);
        double M21 = M12;
        double M22 = p.J + p.m * p.l * p.l;

        double u = controlInput(t);
        double rhs1 = u - p.c * xd - p.k * x + p.m * p.l * Math.sin(th) * thd * thd;
        double rhs2 = -p.b * thd - p.m * p.g * p.l * Math.sin(th);

        double det = M11 * M22 - M12 * M21;
        double xdd = (rhs1 * M22 - rhs2 * M12) / det;
        double thdd = (M11 * rhs2 - M21 * rhs1) / det;

        return new double[]{xd, thd, xdd, thdd};
    }

    static double[] addScaled(double[] a, double[] b, double scale) {
        return new double[]{
            a[0] + scale * b[0],
            a[1] + scale * b[1],
            a[2] + scale * b[2],
            a[3] + scale * b[3]
        };
    }

    static double totalEnergy(double[] s, Params p) {
        double x = s[0];
        double th = s[1];
        double xd = s[2];
        double thd = s[3];

        double T = 0.5 * (p.M + p.m) * xd * xd
                 + p.m * p.l * Math.cos(th) * xd * thd
                 + 0.5 * (p.J + p.m * p.l * p.l) * thd * thd;
        double V = 0.5 * p.k * x * x + p.m * p.g * p.l * (1.0 - Math.cos(th));
        return T + V;
    }

    public static void main(String[] args) throws IOException {
        Params p = new Params();
        double[] s = new double[]{0.05, 0.35, 0.0, 0.0};

        double t0 = 0.0;
        double tf = 10.0;
        double h = 0.001;
        int nSteps = (int)((tf - t0) / h);

        try (PrintWriter out = new PrintWriter(new FileWriter("Chapter18_Lesson2_java_output.csv"))) {
            out.println("t,x,theta,xdot,thetadot,energy");

            double t = t0;
            for (int k = 0; k <= nSteps; k++) {
                out.printf(java.util.Locale.US, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f%n",
                        t, s[0], s[1], s[2], s[3], totalEnergy(s, p));

                double[] k1 = rhs(t, s, p);
                double[] k2 = rhs(t + 0.5 * h, addScaled(s, k1, 0.5 * h), p);
                double[] k3 = rhs(t + 0.5 * h, addScaled(s, k2, 0.5 * h), p);
                double[] k4 = rhs(t + h, addScaled(s, k3, h), p);

                for (int i = 0; i < 4; i++) {
                    s[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
                }
                t += h;
            }
        }

        System.out.println("Simulation finished. Output saved to Chapter18_Lesson2_java_output.csv");
        System.out.println("Compile: javac Chapter18_Lesson2.java");
        System.out.println("Run: java Chapter18_Lesson2");
    }
}
