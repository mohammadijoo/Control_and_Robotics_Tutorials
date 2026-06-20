/*
Chapter14_Lesson4.java
System Dynamics (Control Engineering) — Chapter 14, Lesson 4
Piecewise-Linear Approximations, Saturation, Dead-Zone, and Backlash Models

Compile:
  javac Chapter14_Lesson4.java

Run:
  java Chapter14_Lesson4

Output:
  Chapter14_Lesson4_output.csv
*/

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

public class Chapter14_Lesson4 {

    static double sat(double u, double umax) {
        if (u > umax) return umax;
        if (u < -umax) return -umax;
        return u;
    }

    static double deadZone(double u, double d) {
        if (Math.abs(u) <= d) return 0.0;
        return u - d * ((u > 0.0) ? 1.0 : -1.0);
    }

    static class Backlash {
        final double b;
        double y;
        Backlash(double width, double y0) { b = width; y = y0; }
        double step(double u) {
            final double half = 0.5 * b;
            if (u > y + half) y = u - half;
            else if (u < y - half) y = u + half;
            return y;
        }
    }

    static class PWL {
        final double[] xs;
        final double[] ys;

        PWL(double[] xs, double[] ys) {
            if (xs.length != ys.length || xs.length < 2) {
                throw new IllegalArgumentException("PWL needs >=2 points with equal lengths.");
            }
            for (int i=0; i<xs.length-1; i++) {
                if (!(xs[i] < xs[i+1])) throw new IllegalArgumentException("xs must be strictly increasing.");
            }
            this.xs = Arrays.copyOf(xs, xs.length);
            this.ys = Arrays.copyOf(ys, ys.length);
        }

        double eval(double x) {
            if (x <= xs[0]) return ys[0];
            if (x >= xs[xs.length-1]) return ys[ys.length-1];
            for (int i=0; i<xs.length-1; i++) {
                if (xs[i] <= x && x <= xs[i+1]) {
                    double x0 = xs[i], x1 = xs[i+1];
                    double y0 = ys[i], y1 = ys[i+1];
                    double t = (x - x0) / (x1 - x0);
                    return (1.0 - t)*y0 + t*y1;
                }
            }
            return ys[ys.length-1];
        }
    }

    static double[] plant(double x1, double x2, double u, double wn, double zeta, double b) {
        double dx1 = x2;
        double dx2 = -2.0*zeta*wn*x2 - (wn*wn)*x1 + b*u;
        return new double[]{dx1, dx2};
    }

    static double[] rk4Step(double x1, double x2, double u, double dt) {
        double wn = 5.0, zeta = 0.25, b = 1.0;

        double[] k1 = plant(x1, x2, u, wn, zeta, b);
        double[] k2 = plant(x1 + 0.5*dt*k1[0], x2 + 0.5*dt*k1[1], u, wn, zeta, b);
        double[] k3 = plant(x1 + 0.5*dt*k2[0], x2 + 0.5*dt*k2[1], u, wn, zeta, b);
        double[] k4 = plant(x1 + dt*k3[0], x2 + dt*k3[1], u, wn, zeta, b);

        double xn1 = x1 + (dt/6.0)*(k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]);
        double xn2 = x2 + (dt/6.0)*(k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]);

        return new double[]{xn1, xn2};
    }

    public static void main(String[] args) throws IOException {
        final double T = 6.0;
        final double dt = 1e-3;

        // PD controller
        final double Kp = 18.0, Kd = 4.5;

        // Nonlinearities
        final double uMax = 2.0;
        final double d = 0.25;
        Backlash backlash = new Backlash(0.20, 0.0);

        // Optional PWL map approximating tanh(u)
        double[] xs = new double[]{-3.0,-1.5,-0.5,0.0,0.5,1.5,3.0};
        double[] ys = new double[xs.length];
        for (int i=0; i<xs.length; i++) ys[i] = Math.tanh(xs[i]);
        PWL pwlTanh = new PWL(xs, ys);

        final boolean usePwlMap = false;

        double x1 = 0.0, x2 = 0.0;

        try (FileWriter fw = new FileWriter("Chapter14_Lesson4_output.csv")) {
            fw.write("t,x1,x2,u_cmd,u_deadzone,u_sat,u_backlash\n");
            for (double t=0.0; t <= T + 1e-12; t += dt) {
                double r = 1.0;
                double e = r - x1;
                double uCmd = Kp*e - Kd*x2;

                double uDz  = deadZone(uCmd, d);
                double uSat = sat(uDz, uMax);
                double uMap = usePwlMap ? pwlTanh.eval(uSat) : uSat;
                double uBl  = backlash.step(uMap);

                double[] xn = rk4Step(x1, x2, uBl, dt);
                x1 = xn[0]; x2 = xn[1];

                fw.write(String.format(java.util.Locale.US,
                        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                        t, x1, x2, uCmd, uDz, uSat, uBl));
            }
        }

        System.out.println("Wrote Chapter14_Lesson4_output.csv");
        System.out.println("Plot x1(t) and compare u_cmd vs u_backlash to see nonlinear effects.");
    }
}
