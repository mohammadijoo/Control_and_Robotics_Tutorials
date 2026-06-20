// Chapter18_Lesson3.java
// Port-Hamiltonian mass-spring-damper simulation using RK4
// Compile: javac Chapter18_Lesson3.java
// Run:     java Chapter18_Lesson3

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter18_Lesson3 {
    static class State {
        double q, p;
        State(double q, double p) { this.q = q; this.p = p; }
    }

    static final double m = 1.5;
    static final double k = 12.0;
    static final double d = 0.8;
    static final double omegaU = 1.4;

    static double u(double t) {
        return Math.sin(omegaU * t);
    }

    static double H(State x) {
        return 0.5 * k * x.q * x.q + 0.5 * x.p * x.p / m;
    }

    static double y(State x) {
        return x.p / m;
    }

    static double dissipation(State x) {
        double v = x.p / m;
        return d * v * v;
    }

    static State dynamics(double t, State x) {
        double qdot = x.p / m;
        double pdot = -k * x.q - d * (x.p / m) + u(t);
        return new State(qdot, pdot);
    }

    static State add(State a, State b) {
        return new State(a.q + b.q, a.p + b.p);
    }

    static State scale(double s, State x) {
        return new State(s * x.q, s * x.p);
    }

    static State rk4Step(double t, State x, double h) {
        State k1 = dynamics(t, x);
        State k2 = dynamics(t + 0.5 * h, add(x, scale(0.5 * h, k1)));
        State k3 = dynamics(t + 0.5 * h, add(x, scale(0.5 * h, k2)));
        State k4 = dynamics(t + h, add(x, scale(h, k3)));
        State sum = add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4));
        return add(x, scale(h / 6.0, sum));
    }

    public static void main(String[] args) throws IOException {
        double T = 20.0;
        double h = 0.002;
        int N = (int)(T / h) + 1;

        State x = new State(0.15, 0.0);
        double H0 = H(x);

        double rhsIntegral = 0.0;
        double prevRhs = 0.0;
        boolean first = true;

        try (PrintWriter out = new PrintWriter(new FileWriter("Chapter18_Lesson3_java_output.csv"))) {
            out.println("t,q,p,H,u,y,dissipation,supply");

            for (int i = 0; i < N; i++) {
                double t = i * h;
                double ui = u(t);
                double yi = y(x);
                double diss = dissipation(x);
                double supply = yi * ui;
                double rhs = -diss + supply;

                if (first) {
                    first = false;
                } else {
                    rhsIntegral += 0.5 * h * (prevRhs + rhs);
                }
                prevRhs = rhs;

                out.printf("%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f%n",
                        t, x.q, x.p, H(x), ui, yi, diss, supply);

                if (i < N - 1) {
                    x = rk4Step(t, x, h);
                }
            }
        }

        double Hf = H(x);
        double residual = (Hf - H0) - rhsIntegral;

        System.out.printf("Final state [q, p] = [%.8f, %.8f]%n", x.q, x.p);
        System.out.printf("Initial energy     = %.10f%n", H0);
        System.out.printf("Final energy       = %.10f%n", Hf);
        System.out.printf("Energy balance residual = %.6e%n", residual);
        System.out.println("CSV written to Chapter18_Lesson3_java_output.csv");
    }
}
