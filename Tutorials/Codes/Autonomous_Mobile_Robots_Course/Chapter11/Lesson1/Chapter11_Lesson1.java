// Chapter11_Lesson1.java
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 11: SLAM I — Filter-Based SLAM
Lesson 1: The SLAM Problem Formulation

Minimal EKF-SLAM core (known data association) using EJML.

Dependencies:
  - EJML (org.ejml:ejml-simple)
Build example (Gradle/Maven) is omitted; this file focuses on the core math.
*/

import org.ejml.simple.SimpleMatrix;

public class Chapter11_Lesson1 {

    static double wrapAngle(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    // Unicycle motion model x' = f(x,u)
    static SimpleMatrix motionModel(SimpleMatrix x, SimpleMatrix u) {
        double px = x.get(0), py = x.get(1), th = x.get(2);
        double v = u.get(0), w = u.get(1), dt = u.get(2);

        double px2, py2, th2;
        if (Math.abs(w) < 1e-12) {
            px2 = px + v * dt * Math.cos(th);
            py2 = py + v * dt * Math.sin(th);
            th2 = th;
        } else {
            double thN = th + w * dt;
            px2 = px + (v / w) * (Math.sin(thN) - Math.sin(th));
            py2 = py - (v / w) * (Math.cos(thN) - Math.cos(th));
            th2 = thN;
        }
        SimpleMatrix xp = new SimpleMatrix(3,1);
        xp.set(0, px2);
        xp.set(1, py2);
        xp.set(2, wrapAngle(th2));
        return xp;
    }

    static SimpleMatrix jacMotionWrtState(SimpleMatrix x, SimpleMatrix u) {
        double th = x.get(2);
        double v = u.get(0), w = u.get(1), dt = u.get(2);

        SimpleMatrix Fx = SimpleMatrix.identity(3);
        if (Math.abs(w) < 1e-12) {
            Fx.set(0,2, -v * dt * Math.sin(th));
            Fx.set(1,2,  v * dt * Math.cos(th));
        } else {
            double th2 = th + w * dt;
            Fx.set(0,2, (v / w) * (Math.cos(th2) - Math.cos(th)));
            Fx.set(1,2, (v / w) * (Math.sin(th2) - Math.sin(th)));
        }
        return Fx;
    }

    // z = [range, bearing]
    static SimpleMatrix measModel(SimpleMatrix x, SimpleMatrix m) {
        double px = x.get(0), py = x.get(1), th = x.get(2);
        double dx = m.get(0) - px;
        double dy = m.get(1) - py;
        double r = Math.sqrt(dx*dx + dy*dy);
        double b = wrapAngle(Math.atan2(dy, dx) - th);
        SimpleMatrix z = new SimpleMatrix(2,1);
        z.set(0, r);
        z.set(1, b);
        return z;
    }

    // Returns Hx (2x3) and Hm (2x2) packed as an array
    static SimpleMatrix[] jacMeasWrtPoseLandmark(SimpleMatrix x, SimpleMatrix m) {
        double px = x.get(0), py = x.get(1);
        double dx = m.get(0) - px;
        double dy = m.get(1) - py;
        double q = dx*dx + dy*dy;
        if (q < 1e-12) q = 1e-12;
        double r = Math.sqrt(q);

        SimpleMatrix Hx = new SimpleMatrix(2,3);
        Hx.set(0,0, -dx/r); Hx.set(0,1, -dy/r); Hx.set(0,2, 0.0);
        Hx.set(1,0,  dy/q); Hx.set(1,1, -dx/q); Hx.set(1,2, -1.0);

        SimpleMatrix Hm = new SimpleMatrix(2,2);
        Hm.set(0,0,  dx/r); Hm.set(0,1,  dy/r);
        Hm.set(1,0, -dy/q); Hm.set(1,1,  dx/q);

        return new SimpleMatrix[]{Hx, Hm};
    }

    public static void main(String[] args) {
        // World landmarks (unknown to the filter)
        SimpleMatrix lm0 = new SimpleMatrix(new double[][]{{4.0},{2.0}});
        int N = 3;
        int n = 3 + 2*N;

        // Augmented state y = [x; m1; m2; m3]
        SimpleMatrix y = new SimpleMatrix(n,1);
        y.zero();
        y.set(0, 0.0); y.set(1, 0.0); y.set(2, 0.0);
        // poor map initialization
        y.set(3, 3.0); y.set(4, 1.0);
        y.set(5, 7.0); y.set(6, 0.0);
        y.set(7, 1.0); y.set(8,-2.0);

        SimpleMatrix P = SimpleMatrix.identity(n).scale(1e-3);
        // large map uncertainty
        for (int i=0;i<N;i++) {
            int idx = 3 + 2*i;
            P.set(idx,   idx,   4.0);
            P.set(idx+1, idx+1, 4.0);
        }

        SimpleMatrix Qpose = new SimpleMatrix(new double[][]{
            {0.02*0.02, 0, 0},
            {0, 0.02*0.02, 0},
            {0, 0, Math.pow(Math.PI/180.0,2)}
        });

        SimpleMatrix Rmeas = new SimpleMatrix(new double[][]{
            {0.10*0.10, 0},
            {0, Math.pow(2.0*Math.PI/180.0,2)}
        });

        // One control input
        SimpleMatrix u = new SimpleMatrix(new double[][]{{1.0},{0.10},{1.0}});

        // --- Predict ---
        SimpleMatrix x = y.extractMatrix(0,3,0,1);
        SimpleMatrix Fx = jacMotionWrtState(x, u);
        SimpleMatrix xPred = motionModel(x, u);

        SimpleMatrix F = SimpleMatrix.identity(n);
        F.insertIntoThis(0,0, Fx);

        SimpleMatrix Q = new SimpleMatrix(n,n);
        Q.zero();
        Q.insertIntoThis(0,0, Qpose);

        y.insertIntoThis(0,0, xPred);
        P = F.mult(P).mult(F.transpose()).plus(Q);

        // --- Update on landmark 0 (known association) ---
        int lmId = 0;
        int idx = 3 + 2*lmId;
        SimpleMatrix mi = y.extractMatrix(idx, idx+2, 0, 1);

        // simulate a measurement (range/bearing) close to truth
        SimpleMatrix z = measModel(xPred, lm0);
        z.set(0, z.get(0) + 0.05);
        z.set(1, wrapAngle(z.get(1) + 1.0*Math.PI/180.0));

        SimpleMatrix zHat = measModel(y.extractMatrix(0,3,0,1), mi);
        SimpleMatrix innov = new SimpleMatrix(2,1);
        innov.set(0, z.get(0) - zHat.get(0));
        innov.set(1, wrapAngle(z.get(1) - zHat.get(1)));

        SimpleMatrix[] J = jacMeasWrtPoseLandmark(y.extractMatrix(0,3,0,1), mi);
        SimpleMatrix Hx = J[0];
        SimpleMatrix Hm = J[1];

        SimpleMatrix H = new SimpleMatrix(2,n);
        H.zero();
        H.insertIntoThis(0,0, Hx);
        H.insertIntoThis(0,idx, Hm);

        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(Rmeas);
        SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());

        y = y.plus(K.mult(innov));
        y.set(2, wrapAngle(y.get(2)));

        SimpleMatrix I = SimpleMatrix.identity(n);
        P = (I.minus(K.mult(H))).mult(P).mult((I.minus(K.mult(H))).transpose())
                .plus(K.mult(Rmeas).mult(K.transpose()));

        System.out.println("Updated pose mean: " + y.extractMatrix(0,3,0,1).transpose());
    }
}
