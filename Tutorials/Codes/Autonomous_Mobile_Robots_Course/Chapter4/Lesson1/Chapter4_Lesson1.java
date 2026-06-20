/*
Chapter4_Lesson1.java
Autonomous Mobile Robots - Chapter 4 (Mobile Robot Dynamics Applied)
Lesson 1: When Dynamics Matter for AMR

A minimal RK4 simulation comparing:
  (1) Kinematic unicycle driven by (v_cmd, w_cmd)
  (2) Dynamic model with bounded wheel torques producing (v_dot, w_dot)

Dependencies: none (plain Java)

Build:
  javac Chapter4_Lesson1.java

Run:
  java Chapter4_Lesson1 > out.csv

Author: Course materials generator
*/

import java.io.PrintStream;

public class Chapter4_Lesson1 {

    static class Params {
        double m = 30.0;
        double Iz = 1.2;
        double rw = 0.10;
        double b  = 0.50;
        double bv = 6.0;
        double bw = 0.25;
        double tauMax = 3.0;
        double kv = 8.0;
        double kw = 1.2;
    }

    static double wrapPi(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0*pi);
        if (a < 0.0) a += 2.0*pi;
        return a - pi;
    }

    static double sat(double x, double lo, double hi) {
        return Math.min(Math.max(x, lo), hi);
    }

    static double[] cmdProfile(double t) {
        double vCmd, wCmd;
        if (t <= 5.0) {
            vCmd = 0.2 + 0.18*t;
            wCmd = 0.6;
        } else {
            vCmd = 1.1;
            wCmd = 1.5;
        }
        return new double[]{vCmd, wCmd};
    }

    static double[] kinRhs(double[] x, double[] u) {
        double px = x[0], py = x[1], th = x[2];
        double v = u[0], w = u[1];
        return new double[]{
            v*Math.cos(th),
            v*Math.sin(th),
            w
        };
    }

    static double[] dynRhs(double[] x, double[] u, Params p) {
        double px = x[0], py = x[1], th = x[2], v = x[3], w = x[4];
        double vRef = u[0], wRef = u[1];

        double tauSum  = p.kv*(vRef - v);
        double tauDiff = p.kw*(wRef - w);

        double tauR = 0.5*(tauSum + tauDiff);
        double tauL = 0.5*(tauSum - tauDiff);

        tauR = sat(tauR, -p.tauMax, p.tauMax);
        tauL = sat(tauL, -p.tauMax, p.tauMax);

        double Fx = (tauR + tauL)/p.rw;
        double Mz = (p.b/(2.0*p.rw))*(tauR - tauL);

        double vDot = (Fx - p.bv*v)/p.m;
        double wDot = (Mz - p.bw*w)/p.Iz;

        return new double[]{
            v*Math.cos(th),
            v*Math.sin(th),
            w,
            vDot,
            wDot
        };
    }

    static double[] rk4Step(RhsFunction f, double t, double[] x, double dt, double[] u) {
        double[] k1 = f.eval(t, x, u);
        double[] x2 = new double[x.length];
        for (int i=0;i<x.length;i++) x2[i] = x[i] + 0.5*dt*k1[i];

        double[] k2 = f.eval(t + 0.5*dt, x2, u);
        double[] x3 = new double[x.length];
        for (int i=0;i<x.length;i++) x3[i] = x[i] + 0.5*dt*k2[i];

        double[] k3 = f.eval(t + 0.5*dt, x3, u);
        double[] x4 = new double[x.length];
        for (int i=0;i<x.length;i++) x4[i] = x[i] + dt*k3[i];

        double[] k4 = f.eval(t + dt, x4, u);

        double[] xn = new double[x.length];
        for (int i=0;i<x.length;i++) {
            xn[i] = x[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        }
        return xn;
    }

    interface RhsFunction {
        double[] eval(double t, double[] x, double[] u);
    }

    public static void main(String[] args) {
        double dt = 0.002;
        double T  = 10.0;
        int steps = (int)(T/dt) + 1;

        double[] xk = new double[]{0.0, 0.0, 0.0};
        double[] xd = new double[]{0.0, 0.0, 0.0, 0.0, 0.0};
        Params p = new Params();

        PrintStream out = System.out;
        out.println("t,xk,yk,thk,xd,yd,thd,vd,wd,v_cmd,w_cmd");

        for (int i=0;i<steps;i++) {
            double t = i*dt;
            double[] u = cmdProfile(t);

            if (i % 250 == 0) { // every 0.5 s
                out.printf(java.util.Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                    t, xk[0], xk[1], xk[2], xd[0], xd[1], xd[2], xd[3], xd[4], u[0], u[1]
                );
            }

            // integrate kinematic
            xk = rk4Step((tt, xx, uu) -> kinRhs(xx, uu), t, xk, dt, u);
            xk[2] = wrapPi(xk[2]);

            // integrate dynamic
            final double[] uFinal = u;
            xd = rk4Step((tt, xx, uu) -> dynRhs(xx, uu, p), t, xd, dt, uFinal);
            xd[2] = wrapPi(xd[2]);
        }
    }
}
