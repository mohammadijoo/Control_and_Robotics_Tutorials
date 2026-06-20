// System Dynamics — Chapter 14 (Nonlinear System Dynamics)
// Lesson 1: Sources and Types of Nonlinearities in Engineering Systems
//
// Nonlinear mass–spring–damper with RK4 integration.
// Writes a CSV trajectory.
//
// Compile:
//   javac Chapter14_Lesson1.java
//
// Run:
//   java Chapter14_Lesson1
//
// Output:
//   Chapter14_Lesson1_trace_java.csv

import java.io.FileWriter;
import java.io.IOException;

public class Chapter14_Lesson1 {

    static class Params {
        double m = 1.0;
        double c = 0.4;
        double k = 4.0;
        double k3 = 8.0;
        double Fc = 0.8;
        double vs = 0.02;
        double b = 1.0;
        double umax = 1.5;
    }

    static class State {
        double x;
        double v;
        State(double x, double v) { this.x = x; this.v = v; }
    }

    static double sat(double u, double umax) {
        if (u >  umax) return  umax;
        if (u < -umax) return -umax;
        return u;
    }

    static double coulombSmooth(double v, double Fc, double vs) {
        return Fc * Math.tanh(v / vs);
    }

    static double inputU(double t) {
        return 1.2 * Math.sin(1.0 * t) + 0.3 * Math.sin(3.0 * t);
    }

    static State f(double t, State s, Params p) {
        double u = sat(inputU(t), p.umax);
        double spring = p.k * s.x + p.k3 * s.x * s.x * s.x;
        double fric = p.c * s.v + coulombSmooth(s.v, p.Fc, p.vs);
        double a = (p.b * u - spring - fric) / p.m;
        return new State(s.v, a);
    }

    static State add(State a, State b) {
        return new State(a.x + b.x, a.v + b.v);
    }

    static State mul(double h, State a) {
        return new State(h * a.x, h * a.v);
    }

    public static void main(String[] args) throws IOException {
        Params p = new Params();

        double t0 = 0.0, tf = 20.0;
        double h = 0.005;

        State s = new State(0.4, 0.0);

        try (FileWriter out = new FileWriter("Chapter14_Lesson1_trace_java.csv")) {
            out.write("t,x,xdot\n");

            double t = t0;
            while (t < tf - 1e-12) {
                out.write(t + "," + s.x + "," + s.v + "\n");

                double hh = h;
                if (t + hh > tf) hh = tf - t;

                State k1 = f(t, s, p);
                State k2 = f(t + 0.5*hh, add(s, mul(0.5*hh, k1)), p);
                State k3 = f(t + 0.5*hh, add(s, mul(0.5*hh, k2)), p);
                State k4 = f(t + hh,     add(s, mul(hh,     k3)), p);

                State incr = add(add(k1, mul(2.0, k2)), add(mul(2.0, k3), k4));
                s = add(s, mul(hh/6.0, incr));
                t += hh;
            }
            out.write(tf + "," + s.x + "," + s.v + "\n");
        }

        System.out.println("Wrote: Chapter14_Lesson1_trace_java.csv");
    }
}
