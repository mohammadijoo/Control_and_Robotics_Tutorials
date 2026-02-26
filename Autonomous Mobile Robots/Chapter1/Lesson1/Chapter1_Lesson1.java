// Chapter1_Lesson1.java
// Autonomous Mobile Robots (Control Engineering) — Chapter 1, Lesson 1
// Lesson: What Makes Mobility Different from Manipulation
//
// Minimal Java simulation that contrasts:
//   (i) differential-drive mobile base planar kinematics
//   (ii) planar 2R manipulator kinematics
//
// Robotics-related Java tooling (not required here):
//   - EJML (efficient matrix library) for linear algebra
//   - ROSJava / ros2-java bindings for ROS integration (project-dependent)
//
// Compile:
//   javac Chapter1_Lesson1.java
// Run:
//   java Chapter1_Lesson1
//
// Output:
//   Chapter1_Lesson1_mobile.csv
//   Chapter1_Lesson1_manipulator.csv

import java.io.FileWriter;
import java.io.IOException;

public class Chapter1_Lesson1 {

    static class Pose2 {
        double x, y, theta;
        Pose2(double x, double y, double theta) {
            this.x = x; this.y = y; this.theta = theta;
        }
    }

    static void fk2r(double q1, double q2, double l1, double l2, double[] outXY) {
        double xe = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double ye = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        outXY[0] = xe;
        outXY[1] = ye;
    }

    public static void main(String[] args) throws IOException {
        final double dt = 0.02;
        final double T  = 8.0;
        final int steps = (int)Math.floor(T / dt);

        // Mobile base controls
        final double v = 0.5;
        final double w = 0.35;

        Pose2 p = new Pose2(0.0, 0.0, 0.0);
        try (FileWriter f = new FileWriter("Chapter1_Lesson1_mobile.csv")) {
            f.write("t,x,y,theta\n");
            for (int k = 0; k <= steps; k++) {
                double t = k * dt;
                f.write(t + "," + p.x + "," + p.y + "," + p.theta + "\n");

                // Euler step
                p.x += dt * v * Math.cos(p.theta);
                p.y += dt * v * Math.sin(p.theta);
                p.theta += dt * w;
            }
        }

        // Manipulator joint-rate controls
        double q1 = 0.2, q2 = 0.9;
        final double qdot1 = 0.25;
        final double qdot2 = -0.15;
        final double l1 = 1.0, l2 = 0.7;

        double[] xy = new double[2];
        try (FileWriter f = new FileWriter("Chapter1_Lesson1_manipulator.csv")) {
            f.write("t,q1,q2,xe,ye\n");
            for (int k = 0; k <= steps; k++) {
                double t = k * dt;
                fk2r(q1, q2, l1, l2, xy);
                f.write(t + "," + q1 + "," + q2 + "," + xy[0] + "," + xy[1] + "\n");

                q1 += dt * qdot1;
                q2 += dt * qdot2;
            }
        }

        System.out.println("Wrote CSV files for mobile base and 2R manipulator trajectories.");
    }
}
