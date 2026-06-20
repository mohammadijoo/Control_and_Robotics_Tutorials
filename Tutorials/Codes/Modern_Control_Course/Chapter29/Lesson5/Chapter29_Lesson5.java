/*
Chapter29_Lesson5.java

Slowly time-varying mass-spring-damper example.
No external Java libraries are required.

Compile:
    javac Chapter29_Lesson5.java

Run:
    java Chapter29_Lesson5
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;

public class Chapter29_Lesson5 {
    private static final double EPSILON = 0.03;

    static class State {
        double x1;
        double x2;

        State(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static double mass(double t) {
        return 1.0 + 0.12 * Math.sin(EPSILON * t);
    }

    static double damping(double t) {
        return 0.22 + 0.04 * Math.cos(EPSILON * t);
    }

    static double stiffness(double t) {
        return 2.0 + 0.30 * Math.sin(0.5 * EPSILON * t);
    }

    static double inputForce(double t) {
        return 0.2 * Math.sin(0.7 * t);
    }

    static State derivative(double t, State x) {
        double m = mass(t);
        double c = damping(t);
        double k = stiffness(t);
        double u = inputForce(t);

        double dx1 = x.x2;
        double dx2 = -(k / m) * x.x1 - (c / m) * x.x2 + (1.0 / m) * u;
        return new State(dx1, dx2);
    }

    static State add(State a, State b, double scale) {
        return new State(a.x1 + scale * b.x1, a.x2 + scale * b.x2);
    }

    static State rk4Step(double t, State x, double dt) {
        State k1 = derivative(t, x);
        State k2 = derivative(t + 0.5 * dt, add(x, k1, 0.5 * dt));
        State k3 = derivative(t + 0.5 * dt, add(x, k2, 0.5 * dt));
        State k4 = derivative(t + dt, add(x, k3, dt));

        double nextX1 = x.x1 + (dt / 6.0) * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1);
        double nextX2 = x.x2 + (dt / 6.0) * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2);
        return new State(nextX1, nextX2);
    }

    static double frozenDecayMargin(double t) {
        double m = mass(t);
        double c = damping(t);
        double k = stiffness(t);

        double trace = -c / m;
        double determinant = k / m;
        double discriminant = trace * trace - 4.0 * determinant;

        if (discriminant >= 0.0) {
            double lambda1 = 0.5 * (trace + Math.sqrt(discriminant));
            double lambda2 = 0.5 * (trace - Math.sqrt(discriminant));
            return -Math.max(lambda1, lambda2);
        }

        return -0.5 * trace;
    }

    public static void main(String[] args) throws IOException {
        Locale.setDefault(Locale.US);

        double t0 = 0.0;
        double tf = 160.0;
        double dt = 0.01;
        int steps = (int) ((tf - t0) / dt);

        State x = new State(1.0, 0.0);

        try (PrintWriter writer = new PrintWriter(new FileWriter("Chapter29_Lesson5_java_output.csv"))) {
            writer.println("t,x1,x2,m,c,k,alpha");

            for (int i = 0; i <= steps; i++) {
                double t = t0 + i * dt;
                writer.printf(
                    "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f%n",
                    t, x.x1, x.x2, mass(t), damping(t), stiffness(t), frozenDecayMargin(t)
                );

                if (i < steps) {
                    x = rk4Step(t, x, dt);
                }
            }
        }

        System.out.printf("Final state: x1 = %.8f, x2 = %.8f%n", x.x1, x.x2);
        System.out.println("CSV written to Chapter29_Lesson5_java_output.csv");
    }
}
