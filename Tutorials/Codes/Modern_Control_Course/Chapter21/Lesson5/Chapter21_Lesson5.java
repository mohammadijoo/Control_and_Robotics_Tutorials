/*
Chapter21_Lesson5.java
Scratch Java simulation of minimum-phase and non-minimum-phase zeros.

Compile and run:
    javac Chapter21_Lesson5.java
    java Chapter21_Lesson5

The model uses the controllable canonical realization for denominator
s^2 + 5s + 6 and compares numerator 6s+6 with numerator -6s+6.
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter21_Lesson5 {
    static class State {
        double x1;
        double x2;
        State(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static State derivative(State x, double u) {
        return new State(x.x2, -6.0 * x.x1 - 5.0 * x.x2 + u);
    }

    static State addScaled(State x, State k, double h) {
        return new State(x.x1 + h * k.x1, x.x2 + h * k.x2);
    }

    static State rk4Step(State x, double u, double dt) {
        State k1 = derivative(x, u);
        State k2 = derivative(addScaled(x, k1, dt / 2.0), u);
        State k3 = derivative(addScaled(x, k2, dt / 2.0), u);
        State k4 = derivative(addScaled(x, k3, dt), u);
        return new State(
            x.x1 + dt * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1) / 6.0,
            x.x2 + dt * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2) / 6.0
        );
    }

    static double output(State x, double b0, double b1) {
        return b0 * x.x1 + b1 * x.x2;
    }

    static double zeroLocation(double b0, double b1) {
        return -b0 / b1;
    }

    public static void main(String[] args) throws IOException {
        double dt = 0.002;
        double tf = 8.0;
        double u = 1.0;

        double b0Min = 6.0;
        double b1Min = 6.0;    // numerator 6s + 6; zero at -1
        double b0Nmp = 6.0;
        double b1Nmp = -6.0;   // numerator -6s + 6; zero at +1

        System.out.println("Minimum-phase zero: " + zeroLocation(b0Min, b1Min));
        System.out.println("Non-minimum-phase zero: " + zeroLocation(b0Nmp, b1Nmp));
        System.out.println("Common poles: -2, -3");

        State xMin = new State(0.0, 0.0);
        State xNmp = new State(0.0, 0.0);
        double negativeArea = 0.0;

        try (PrintWriter csv = new PrintWriter(new FileWriter("Chapter21_Lesson5_java_step_response.csv"))) {
            csv.println("t,y_minimum_phase,y_nonminimum_phase");
            int steps = (int) Math.round(tf / dt);
            for (int i = 0; i <= steps; i++) {
                double t = i * dt;
                double yMin = output(xMin, b0Min, b1Min);
                double yNmp = output(xNmp, b0Nmp, b1Nmp);
                csv.printf("%.6f,%.10f,%.10f%n", t, yMin, yNmp);
                if (yNmp < 0.0) {
                    negativeArea += -yNmp * dt;
                }
                xMin = rk4Step(xMin, u, dt);
                xNmp = rk4Step(xNmp, u, dt);
            }
        }

        System.out.println("Wrote Chapter21_Lesson5_java_step_response.csv");
        System.out.println("Approximate negative undershoot area: " + negativeArea);
    }
}
