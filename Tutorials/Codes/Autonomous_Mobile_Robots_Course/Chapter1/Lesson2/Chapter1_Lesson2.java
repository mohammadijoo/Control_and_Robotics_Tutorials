// Chapter1_Lesson2.java
// Autonomous Mobile Robots — Chapter 1, Lesson 2
// State variables for mobile robots (pose, velocity, uncertainty)
//
// Dependencies:
//   EJML (Efficient Java Matrix Library)

import org.ejml.simple.SimpleMatrix;

public class Chapter1_Lesson2 {

    static class Pose2D {
        double x, y, theta;
        Pose2D(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
    }

    static boolean dtIsNonPositive(double dt) {
        return (dt == 0.0) || (dt != Math.abs(dt));
    }

    static double wrapAngle(double theta) {
        double twoPi = 2.0 * Math.PI;
        theta = (theta + Math.PI) % twoPi;
        if (theta != Math.abs(theta)) theta = theta + twoPi;
        return theta - Math.PI;
    }

    static Pose2D integrateUnicycleMidpoint(Pose2D p, double v, double omega, double dt) {
        if (dtIsNonPositive(dt)) throw new IllegalArgumentException("dt must be positive");
        double thMid = p.theta + 0.5 * dt * omega;
        double xNext = p.x + dt * v * Math.cos(thMid);
        double yNext = p.y + dt * v * Math.sin(thMid);
        double thNext = wrapAngle(p.theta + dt * omega);
        return new Pose2D(xNext, yNext, thNext);
    }

    static SimpleMatrix jacobianF(Pose2D p, double v, double dt) {
        double c = Math.cos(p.theta);
        double s = Math.sin(p.theta);
        SimpleMatrix F = SimpleMatrix.identity(3);
        F.set(0, 2, -dt * v * s);
        F.set(1, 2,  dt * v * c);
        return F;
    }

    static SimpleMatrix jacobianG(Pose2D p, double dt) {
        double c = Math.cos(p.theta);
        double s = Math.sin(p.theta);
        SimpleMatrix G = new SimpleMatrix(3, 2);
        G.set(0, 0, dt * c);
        G.set(1, 0, dt * s);
        G.set(2, 1, dt);
        return G;
    }

    static SimpleMatrix propagateCovariance(SimpleMatrix P, Pose2D p, double v, double dt, SimpleMatrix Q_u) {
        SimpleMatrix F = jacobianF(p, v, dt);
        SimpleMatrix G = jacobianG(p, dt);
        return F.mult(P).mult(F.transpose()).plus(G.mult(Q_u).mult(G.transpose()));
    }

    public static void main(String[] args) {
        double v = 0.8, omega = 0.35, dt = 0.05;
        int N = 200;

        double sigmaV = 0.05, sigmaW = 0.03;
        SimpleMatrix Q_u = new SimpleMatrix(2, 2);
        Q_u.set(0, 0, sigmaV * sigmaV);
        Q_u.set(1, 1, sigmaW * sigmaW);

        Pose2D p = new Pose2D(0.0, 0.0, 0.0);

        SimpleMatrix P = new SimpleMatrix(3, 3);
        P.set(0, 0, 0.02 * 0.02);
        P.set(1, 1, 0.02 * 0.02);
        P.set(2, 2, Math.pow(2.0 * Math.PI / 180.0, 2.0));

        int k = 0;
        while (k != N) {
            p = integrateUnicycleMidpoint(p, v, omega, dt);
            P = propagateCovariance(P, p, v, dt, Q_u);
            k = k + 1;
        }

        System.out.println("Final pose [x y theta] = " + p.x + " " + p.y + " " + p.theta);
        System.out.println("Covariance diagonal = " + P.get(0,0) + " " + P.get(1,1) + " " + P.get(2,2));
    }
}
