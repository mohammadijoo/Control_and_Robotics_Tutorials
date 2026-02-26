/*
Chapter2_Lesson3.java
Autonomous Mobile Robots — Chapter 2, Lesson 3
Car-Like / Ackermann Steering Kinematics

Compile & run:
  javac Chapter2_Lesson3.java
  java Chapter2_Lesson3

Common AMR Java libraries:
- EJML (linear algebra)
- ROSJava (ROS 1; community-maintained)
- JavaCPP presets for native libs (e.g., OpenCV) when needed

This example is self-contained and focuses on kinematics.
*/

public class Chapter2_Lesson3 {

    public static class Pose2D {
        public double x, y, theta;
        public Pose2D(double x, double y, double theta) {
            this.x = x; this.y = y; this.theta = theta;
        }
        @Override
        public String toString() {
            return String.format("Pose2D(x=%.6f, y=%.6f, theta=%.6f)", x, y, theta);
        }
    }

    public static double wrapToPi(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    public static double curvatureFromSteering(double delta, double L) {
        return Math.tan(delta) / L;
    }

    // Return {deltaLeft, deltaRight}
    public static double[] ackermannWheelAnglesFromVirtual(double delta, double L, double W) {
        double kappa = curvatureFromSteering(delta, L);
        if (Math.abs(kappa) < 1e-12) return new double[]{0.0, 0.0};

        double R = 1.0 / kappa; // signed
        double dl = Math.atan2(L, (R - W / 2.0));
        double dr = Math.atan2(L, (R + W / 2.0));
        return new double[]{dl, dr};
    }

    public static Pose2D stepBicycleExact(Pose2D p, double v, double delta, double L, double dt) {
        double omega = v * Math.tan(delta) / L;

        if (Math.abs(omega) < 1e-10) {
            double x2 = p.x + dt * v * Math.cos(p.theta);
            double y2 = p.y + dt * v * Math.sin(p.theta);
            double t2 = wrapToPi(p.theta + dt * omega);
            return new Pose2D(x2, y2, t2);
        }

        double th2 = p.theta + omega * dt;
        double x2 = p.x + (v / omega) * (Math.sin(th2) - Math.sin(p.theta));
        double y2 = p.y + (v / omega) * (-Math.cos(th2) + Math.cos(p.theta));
        double t2 = wrapToPi(th2);
        return new Pose2D(x2, y2, t2);
    }

    public static void main(String[] args) {
        double L = 2.7;
        double W = 1.6;

        Pose2D p = new Pose2D(0.0, 0.0, 0.0);

        // controls: (v, delta, dt)
        double[][] controls = new double[][]{
                {2.0, Math.toRadians(15.0), 1.0},
                {2.0, Math.toRadians(15.0), 1.0},
                {2.0, Math.toRadians(0.0), 1.0},
                {2.0, Math.toRadians(0.0), 1.0}
        };

        for (double[] u : controls) {
            p = stepBicycleExact(p, u[0], u[1], L, u[2]);
        }

        System.out.println("Final pose (exact): " + p);

        double deltaVirtual = Math.toRadians(15.0);
        double[] lr = ackermannWheelAnglesFromVirtual(deltaVirtual, L, W);
        System.out.printf("Virtual delta = %.6f rad%n", deltaVirtual);
        System.out.printf("Ackermann delta_left  = %.6f rad%n", lr[0]);
        System.out.printf("Ackermann delta_right = %.6f rad%n", lr[1]);
    }
}
