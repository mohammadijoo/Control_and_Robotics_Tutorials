/*
Chapter22_Lesson5.java

From-scratch Java simulation of practical state-feedback limitations:
amplitude saturation and actuator rate limiting.

Compile:
    javac Chapter22_Lesson5.java

Run:
    java Chapter22_Lesson5

The program writes Chapter22_Lesson5_java_output.csv.
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter22_Lesson5 {
    static double saturate(double value, double lower, double upper) {
        return Math.min(Math.max(value, lower), upper);
    }

    static double rateLimit(double requested, double previous, double maxRate, double dt) {
        double step = maxRate * dt;
        return saturate(requested, previous - step, previous + step);
    }

    static double[] dynamics(double[] x, double u) {
        return new double[] {x[1], u};
    }

    static double[] addScaled(double[] x, double[] dx, double scale) {
        return new double[] {x[0] + scale * dx[0], x[1] + scale * dx[1]};
    }

    public static void main(String[] args) throws IOException {
        double k1 = 6.0;
        double k2 = 5.0;
        double umax = 1.0;
        double maxRate = 8.0;
        double dt = 0.001;
        double tf = 6.0;
        int nSteps = (int) Math.round(tf / dt);

        double[] x = new double[] {1.2, 0.0};
        double uPrev = 0.0;

        try (PrintWriter file = new PrintWriter(new FileWriter("Chapter22_Lesson5_java_output.csv"))) {
            file.println("time,x1,x2,u_command,u_actual");

            for (int i = 0; i <= nSteps; i++) {
                double time = i * dt;
                double uCommand = -(k1 * x[0] + k2 * x[1]);
                double uSat = saturate(uCommand, -umax, umax);
                double u = rateLimit(uSat, uPrev, maxRate, dt);
                uPrev = u;

                file.printf("%.8f,%.12f,%.12f,%.12f,%.12f%n",
                    time, x[0], x[1], uCommand, u);

                double[] q1 = dynamics(x, u);
                double[] q2 = dynamics(addScaled(x, q1, 0.5 * dt), u);
                double[] q3 = dynamics(addScaled(x, q2, 0.5 * dt), u);
                double[] q4 = dynamics(addScaled(x, q3, dt), u);

                x[0] += (dt / 6.0) * (q1[0] + 2.0 * q2[0] + 2.0 * q3[0] + q4[0]);
                x[1] += (dt / 6.0) * (q1[1] + 2.0 * q2[1] + 2.0 * q3[1] + q4[1]);
            }
        }

        System.out.println("Finished. CSV written to Chapter22_Lesson5_java_output.csv");
    }
}
