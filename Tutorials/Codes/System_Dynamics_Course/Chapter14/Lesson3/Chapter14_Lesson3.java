/*
Chapter 14 - Nonlinear System Dynamics
Lesson 3 - Linearization vs. True Nonlinear Behavior: When Linear Models Fail

File: Chapter14_Lesson3.java
Build & run:
  javac Chapter14_Lesson3.java
  java Chapter14_Lesson3

Outputs CSV: Chapter14_Lesson3_ExampleB_java.csv
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter14_Lesson3 {

    // Example B nonlinear dynamics
    static double[] fNL(double t, double[] z) {
        double x = z[0], y = z[1];
        double r2 = x*x + y*y;
        return new double[] { -y + x*r2, x + y*r2 };
    }

    // Linearization at origin: zdot = A z, A=[[0,-1],[1,0]]
    static double[] fLIN(double t, double[] z) {
        double x = z[0], y = z[1];
        return new double[] { -y, x };
    }

    static double[] add(double[] a, double[] b) {
        return new double[] { a[0] + b[0], a[1] + b[1] };
    }

    static double[] mul(double s, double[] a) {
        return new double[] { s * a[0], s * a[1] };
    }

    static double norm2(double[] a) {
        return Math.sqrt(a[0]*a[0] + a[1]*a[1]);
    }

    interface F {
        double[] eval(double t, double[] x);
    }

    static double[] rk4Step(F f, double t, double[] x, double h) {
        double[] k1 = f.eval(t, x);
        double[] k2 = f.eval(t + 0.5*h, add(x, mul(0.5*h, k1)));
        double[] k3 = f.eval(t + 0.5*h, add(x, mul(0.5*h, k2)));
        double[] k4 = f.eval(t + h, add(x, mul(h, k3)));
        double[] sum = add(k1, add(mul(2.0, k2), add(mul(2.0, k3), k4)));
        return add(x, mul(h/6.0, sum));
    }

    public static void main(String[] args) throws IOException {
        double t0 = 0.0, tf = 25.0, h = 1e-3;
        int nSteps = (int)Math.ceil((tf - t0) / h);

        double[] z0 = new double[] { 0.2, 0.0 };
        double[] zNL = new double[] { z0[0], z0[1] };
        double[] zLIN = new double[] { z0[0], z0[1] };

        PrintWriter out = new PrintWriter(new FileWriter("Chapter14_Lesson3_ExampleB_java.csv"));
        out.println("t,x_lin,y_lin,r_lin,x_nl,y_nl,r_nl");

        double t = t0;
        for (int k = 0; k <= nSteps; k++) {
            double rlin = norm2(zLIN);
            double rnl  = norm2(zNL);
            out.printf(java.util.Locale.US, "%.6f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f%n",
                    t, zLIN[0], zLIN[1], rlin, zNL[0], zNL[1], rnl);

            if (k == nSteps) break;
            zLIN = rk4Step(Chapter14_Lesson3::fLIN, t, zLIN, h);
            zNL  = rk4Step(Chapter14_Lesson3::fNL,  t, zNL,  h);
            t += h;
        }

        out.close();
        System.out.println("Saved: Chapter14_Lesson3_ExampleB_java.csv");
        System.out.println("Final r_lin=" + norm2(zLIN) + "  r_nl=" + norm2(zNL));
    }
}
