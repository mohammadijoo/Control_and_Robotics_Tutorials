// Chapter7_Lesson1.java
// Chapter 7 - Lesson 1: EKF Localization Pipeline (mobile framing)
//
// Minimal EKF localization demo (2D) in Java using EJML.
// Dependency: org.ejml:ejml-simple
//
// Compile/run (example with Gradle or Maven):
// - Add EJML dependency and run this class' main().

import org.ejml.simple.SimpleMatrix;

import java.util.Random;

public class Chapter7_Lesson1 {

    static class Landmark {
        final double mx, my;
        Landmark(double mx, double my) { this.mx = mx; this.my = my; }
    }

    static double wrapAngle(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    static SimpleMatrix motionModel(SimpleMatrix x, SimpleMatrix u, double dt) {
        double px = x.get(0), py = x.get(1), th = x.get(2);
        double v = u.get(0), w = u.get(1);
        SimpleMatrix nx = new SimpleMatrix(3,1);
        nx.set(0, px + dt * v * Math.cos(th));
        nx.set(1, py + dt * v * Math.sin(th));
        nx.set(2, wrapAngle(th + dt * w));
        return nx;
    }

    static SimpleMatrix jacobianF(SimpleMatrix x, SimpleMatrix u, double dt) {
        double th = x.get(2);
        double v  = u.get(0);
        SimpleMatrix F = SimpleMatrix.identity(3);
        F.set(0,2, -dt * v * Math.sin(th));
        F.set(1,2,  dt * v * Math.cos(th));
        return F;
    }

    static SimpleMatrix jacobianL(SimpleMatrix x, double dt) {
        double th = x.get(2);
        SimpleMatrix L = new SimpleMatrix(3,2);
        L.set(0,0, dt * Math.cos(th));
        L.set(1,0, dt * Math.sin(th));
        L.set(2,1, dt);
        return L;
    }

    static SimpleMatrix measRangeBearing(SimpleMatrix x, Landmark lm) {
        double dx = lm.mx - x.get(0);
        double dy = lm.my - x.get(1);
        double r = Math.sqrt(dx*dx + dy*dy);
        double b = wrapAngle(Math.atan2(dy, dx) - x.get(2));
        SimpleMatrix z = new SimpleMatrix(2,1);
        z.set(0, r);
        z.set(1, b);
        return z;
    }

    static SimpleMatrix jacobianH(SimpleMatrix x, Landmark lm) {
        double dx = lm.mx - x.get(0);
        double dy = lm.my - x.get(1);
        double q  = dx*dx + dy*dy;
        double eps = 1e-12;
        if (q < eps) q = eps;
        double r = Math.sqrt(q);

        SimpleMatrix H = new SimpleMatrix(2,3);
        // range
        H.set(0,0, -dx / r);
        H.set(0,1, -dy / r);
        // bearing
        H.set(1,0,  dy / q);
        H.set(1,1, -dx / q);
        H.set(1,2, -1.0);
        return H;
    }

    static int nearestLandmarkIndex(SimpleMatrix x, Landmark[] lms) {
        int best = 0;
        double bestd = 1e100;
        for (int i=0;i<lms.length;i++) {
            double dx = lms[i].mx - x.get(0);
            double dy = lms[i].my - x.get(1);
            double d = Math.sqrt(dx*dx + dy*dy);
            if (d < bestd) { bestd = d; best = i; }
        }
        return best;
    }

    public static void main(String[] args) {
        Landmark[] lms = new Landmark[] {
                new Landmark(5.0, 0.0),
                new Landmark(5.0, 5.0),
                new Landmark(0.0, 5.0),
                new Landmark(-3.0, 2.0)
        };

        double dt = 0.1;
        int N = 250;

        // Assumed process noise (tuning)
        SimpleMatrix Q = new SimpleMatrix(2,2);
        Q.set(0,0, 0.08*0.08);
        Q.set(1,1, 0.05*0.05);

        // Measurement noise
        SimpleMatrix R = new SimpleMatrix(2,2);
        R.set(0,0, 0.15*0.15);
        R.set(1,1, Math.pow(2.0*Math.PI/180.0, 2));

        // True noise (on control)
        double sigmaVTrue = 0.05;
        double sigmaWTrue = 0.03;

        Random rng = new Random(7);

        // Initial truth and estimate
        SimpleMatrix xTrue = new SimpleMatrix(3,1);
        SimpleMatrix xEst  = new SimpleMatrix(3,1);
        xEst.set(0, -0.2); xEst.set(1, 0.1); xEst.set(2, 0.05);

        SimpleMatrix P = new SimpleMatrix(3,3);
        P.set(0,0, 0.5*0.5);
        P.set(1,1, 0.5*0.5);
        P.set(2,2, Math.pow(10.0*Math.PI/180.0, 2));

        double sumPos2 = 0.0, sumTh2 = 0.0;

        for (int k=0;k<N;k++) {
            double vCmd = 0.8 + 0.2 * Math.sin(0.04 * k);
            double wCmd = 0.25 + 0.05 * Math.cos(0.03 * k);

            SimpleMatrix uCmd = new SimpleMatrix(2,1);
            uCmd.set(0, vCmd); uCmd.set(1, wCmd);

            // Truth uses noisy control
            double vTrue = vCmd + rng.nextGaussian() * sigmaVTrue;
            double wTrue = wCmd + rng.nextGaussian() * sigmaWTrue;
            SimpleMatrix uTrue = new SimpleMatrix(2,1);
            uTrue.set(0, vTrue); uTrue.set(1, wTrue);

            xTrue = motionModel(xTrue, uTrue, dt);

            // EKF predict
            SimpleMatrix F = jacobianF(xEst, uCmd, dt);
            SimpleMatrix L = jacobianL(xEst, dt);
            xEst = motionModel(xEst, uCmd, dt);
            P = F.mult(P).mult(F.transpose()).plus(L.mult(Q).mult(L.transpose()));
            P = P.plus(P.transpose()).scale(0.5);

            // Measurement (nearest landmark)
            int idx = nearestLandmarkIndex(xTrue, lms);
            Landmark lm = lms[idx];

            SimpleMatrix zTrue = measRangeBearing(xTrue, lm);
            double zR = zTrue.get(0) + rng.nextGaussian() * Math.sqrt(R.get(0,0));
            double zB = wrapAngle(zTrue.get(1) + rng.nextGaussian() * Math.sqrt(R.get(1,1)));
            SimpleMatrix zMeas = new SimpleMatrix(2,1);
            zMeas.set(0, zR); zMeas.set(1, zB);

            // EKF update
            SimpleMatrix H = jacobianH(xEst, lm);
            SimpleMatrix zHat = measRangeBearing(xEst, lm);
            SimpleMatrix y = zMeas.minus(zHat);
            y.set(1, wrapAngle(y.get(1)));

            SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
            SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());

            xEst = xEst.plus(K.mult(y));
            xEst.set(2, wrapAngle(xEst.get(2)));

            SimpleMatrix I = SimpleMatrix.identity(3);
            P = (I.minus(K.mult(H))).mult(P).mult(I.minus(K.mult(H)).transpose())
                    .plus(K.mult(R).mult(K.transpose()));
            P = P.plus(P.transpose()).scale(0.5);

            // Errors
            double ex = xEst.get(0) - xTrue.get(0);
            double ey = xEst.get(1) - xTrue.get(1);
            double eth = wrapAngle(xEst.get(2) - xTrue.get(2));
            sumPos2 += ex*ex + ey*ey;
            sumTh2 += eth*eth;
        }

        double rmsPos = Math.sqrt(sumPos2 / (double)N);
        double rmsTh  = Math.sqrt(sumTh2 / (double)N);

        System.out.println("RMS position error [m]: " + rmsPos);
        System.out.println("RMS heading error [rad]: " + rmsTh);
    }
}
