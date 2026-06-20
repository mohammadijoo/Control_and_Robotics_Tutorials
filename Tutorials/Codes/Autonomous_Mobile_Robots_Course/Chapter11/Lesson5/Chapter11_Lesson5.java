/*
Chapter 11 - Lesson 5: Lab: EKF-SLAM in a Small World
Autonomous Mobile Robots (Control Engineering)

Minimal Java EKF-SLAM demo using EJML for matrices.

Build/Run (example with Maven/Gradle classpath set for EJML):
  javac -cp ejml-all-0.43.jar Chapter11_Lesson5.java
  java  -cp .;ejml-all-0.43.jar Chapter11_Lesson5

This code is intentionally compact for instructional clarity.
*/

import java.util.*;
import org.ejml.simple.SimpleMatrix;

public class Chapter11_Lesson5 {

    static double wrapPi(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0 * pi);
        if (a < 0.0) a += 2.0 * pi;
        return a - pi;
    }

    static class Obs {
        int id;
        double r;
        double b;
        Obs(int id, double r, double b) { this.id = id; this.r = r; this.b = b; }
    }

    static class EKFSLAM {
        SimpleMatrix mu; // (3+2N)x1
        SimpleMatrix P;  // (3+2N)x(3+2N)
        SimpleMatrix Q;  // 2x2
        SimpleMatrix R;  // 2x2
        Map<Integer,Integer> seen = new HashMap<>();
        int nL = 0;

        EKFSLAM(SimpleMatrix Q, SimpleMatrix R) {
            this.Q = Q.copy();
            this.R = R.copy();
            mu = new SimpleMatrix(3,1);
            P  = SimpleMatrix.identity(3).scale(1e-6);
        }

        double[] motion(double[] xr, double v, double w, double dt) {
            double x = xr[0], y = xr[1], th = xr[2];
            double x2 = x + v * dt * Math.cos(th);
            double y2 = y + v * dt * Math.sin(th);
            double th2 = wrapPi(th + w * dt);
            return new double[]{x2, y2, th2};
        }

        SimpleMatrix Fjac(double[] xr, double v, double dt) {
            double th = xr[2];
            SimpleMatrix F = SimpleMatrix.identity(3);
            F.set(0,2, -v*dt*Math.sin(th));
            F.set(1,2,  v*dt*Math.cos(th));
            return F;
        }

        SimpleMatrix Ljac(double[] xr, double dt) {
            double th = xr[2];
            SimpleMatrix L = new SimpleMatrix(3,2);
            L.set(0,0, dt*Math.cos(th));
            L.set(1,0, dt*Math.sin(th));
            L.set(2,1, dt);
            return L;
        }

        double[] meas(double[] xr, double[] lm) {
            double dx = lm[0] - xr[0];
            double dy = lm[1] - xr[1];
            double r = Math.sqrt(dx*dx + dy*dy);
            double b = wrapPi(Math.atan2(dy, dx) - xr[2]);
            return new double[]{r, b};
        }

        void predict(double v, double w, double dt) {
            double[] xr = new double[]{mu.get(0), mu.get(1), mu.get(2)};
            double[] xr2 = motion(xr, v, w, dt);

            SimpleMatrix F = Fjac(xr, v, dt);
            SimpleMatrix L = Ljac(xr, dt);
            SimpleMatrix Qx = L.mult(Q).mult(L.transpose());

            int dim = 3 + 2*nL;
            SimpleMatrix Fbig = SimpleMatrix.identity(dim);
            Fbig.insertIntoThis(0,0, F);

            mu.set(0, xr2[0]);
            mu.set(1, xr2[1]);
            mu.set(2, xr2[2]);

            P = Fbig.mult(P).mult(Fbig.transpose());
            // add Qx into robot block
            for (int i = 0; i != 3; ++i) {
                for (int j = 0; j != 3; ++j) {
                    P.set(i,j, P.get(i,j) + Qx.get(i,j));
                }
            }
            P = P.plus(P.transpose()).scale(0.5);
        }

        void augmentLandmark(int id, double r, double b) {
            double x = mu.get(0), y = mu.get(1), th = mu.get(2);
            double lx = x + r * Math.cos(th + b);
            double ly = y + r * Math.sin(th + b);

            int oldDim = mu.numRows();
            SimpleMatrix mu2 = new SimpleMatrix(oldDim + 2, 1);
            mu2.insertIntoThis(0,0, mu);
            mu2.set(oldDim,   0, lx);
            mu2.set(oldDim+1, 0, ly);
            mu = mu2;

            double c = Math.cos(th + b);
            double s = Math.sin(th + b);

            SimpleMatrix Gx = new SimpleMatrix(2,3);
            Gx.set(0,0, 1.0); Gx.set(0,2, -r*s);
            Gx.set(1,1, 1.0); Gx.set(1,2,  r*c);

            SimpleMatrix Gz = new SimpleMatrix(2,2);
            Gz.set(0,0, c);    Gz.set(0,1, -r*s);
            Gz.set(1,0, s);    Gz.set(1,1,  r*c);

            SimpleMatrix Prr = P.extractMatrix(0,3,0,3);
            SimpleMatrix Pmm = Gx.mult(Prr).mult(Gx.transpose()).plus(Gz.mult(R).mult(Gz.transpose()));

            SimpleMatrix P2 = new SimpleMatrix(oldDim + 2, oldDim + 2);
            P2.insertIntoThis(0,0, P);

            SimpleMatrix Px_r = P.extractMatrix(0, oldDim, 0, 3);
            SimpleMatrix Pxm = Px_r.mult(Gx.transpose()); // oldDim x 2
            P2.insertIntoThis(0, oldDim, Pxm);
            P2.insertIntoThis(oldDim, 0, Pxm.transpose());
            P2.insertIntoThis(oldDim, oldDim, Pmm);

            P = P2.plus(P2.transpose()).scale(0.5);

            seen.put(id, nL);
            nL += 1;
        }

        void updateOne(int id, double rMeas, double bMeas) {
            if (!seen.containsKey(id)) {
                augmentLandmark(id, rMeas, bMeas);
                return;
            }
            int idx = seen.get(id);
            double[] xr = new double[]{mu.get(0), mu.get(1), mu.get(2)};
            double[] lm = new double[]{mu.get(3 + 2*idx), mu.get(3 + 2*idx + 1)};

            double[] zhat = meas(xr, lm);
            double y0 = rMeas - zhat[0];
            double y1 = wrapPi(bMeas - zhat[1]);

            double dx = lm[0] - xr[0];
            double dy = lm[1] - xr[1];
            double q = dx*dx + dy*dy;
            double rr = Math.sqrt(Math.max(q, 1e-12));

            SimpleMatrix Hxr = new SimpleMatrix(2,3);
            Hxr.set(0,0, -dx/rr); Hxr.set(0,1, -dy/rr);
            Hxr.set(1,0,  dy/q);  Hxr.set(1,1, -dx/q); Hxr.set(1,2, -1.0);

            SimpleMatrix Hlm = new SimpleMatrix(2,2);
            Hlm.set(0,0, dx/rr); Hlm.set(0,1, dy/rr);
            Hlm.set(1,0, -dy/q); Hlm.set(1,1, dx/q);

            int dim = 3 + 2*nL;
            SimpleMatrix H = new SimpleMatrix(2, dim);
            H.insertIntoThis(0,0, Hxr);
            H.insertIntoThis(0, 3 + 2*idx, Hlm);

            SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
            SimpleMatrix Sinv = S.invert();
            SimpleMatrix y = new SimpleMatrix(2,1);
            y.set(0,0,y0);
            y.set(1,0,y1);

            double d2 = y.transpose().mult(Sinv).mult(y).get(0,0);
            if (d2 > 9.210) return;

            SimpleMatrix K = P.mult(H.transpose()).mult(Sinv);
            SimpleMatrix I = SimpleMatrix.identity(dim);

            mu = mu.plus(K.mult(y));
            mu.set(2, 0, wrapPi(mu.get(2)));
            P = (I.minus(K.mult(H))).mult(P).mult((I.minus(K.mult(H))).transpose())
                    .plus(K.mult(R).mult(K.transpose()));
            P = P.plus(P.transpose()).scale(0.5);
        }
    }

    public static void main(String[] args) {
        // Landmarks (id -> [x,y])
        Map<Integer, double[]> landmarks = new HashMap<>();
        landmarks.put(1, new double[]{4.0, 3.5});
        landmarks.put(2, new double[]{8.5, 1.5});
        landmarks.put(3, new double[]{7.5, 7.5});

        double dt = 0.1;
        int T = 120;

        SimpleMatrix Q = new SimpleMatrix(2,2);
        Q.set(0,0, 0.05*0.05);
        Q.set(1,1, 0.03*0.03);

        double sigB = Math.toRadians(2.5);
        SimpleMatrix R = new SimpleMatrix(2,2);
        R.set(0,0, 0.12*0.12);
        R.set(1,1, sigB*sigB);

        EKFSLAM ekf = new EKFSLAM(Q, R);
        ekf.P = new SimpleMatrix(3,3);
        ekf.P.set(0,0, 0.05*0.05);
        ekf.P.set(1,1, 0.05*0.05);
        ekf.P.set(2,2, Math.toRadians(5.0)*Math.toRadians(5.0));

        double[] xgt = new double[]{1.0, 1.0, Math.toRadians(20.0)};

        for (int k = 0; k != T; ++k) {
            double v = 0.7 + 0.15 * Math.sin(0.08 * k);
            double w = 0.35 * Math.sin(0.05 * k);

            // propagate truth
            xgt = ekf.motion(xgt, v, w, dt);

            // predict
            ekf.predict(v, w, dt);

            // observations within range and FOV
            for (Map.Entry<Integer, double[]> e : landmarks.entrySet()) {
                int id = e.getKey();
                double[] lm = e.getValue();
                double[] ztrue = ekf.meas(xgt, lm);
                if (ztrue[0] < 6.0 && Math.abs(ztrue[1]) < Math.toRadians(70.0)) {
                    // deterministic pseudo-noise
                    double rMeas = ztrue[0] + 0.02 * Math.sin(0.31*k + id);
                    double bMeas = wrapPi(ztrue[1] + Math.toRadians(1.0) * Math.sin(0.19*k + 2*id));
                    ekf.updateOne(id, rMeas, bMeas);
                }
            }
        }

        System.out.println("Final estimate [x y theta]: " +
                ekf.mu.get(0) + " " + ekf.mu.get(1) + " " + ekf.mu.get(2));
        System.out.println("Final ground truth [x y theta]: " +
                xgt[0] + " " + xgt[1] + " " + xgt[2]);

        for (Map.Entry<Integer,Integer> e : ekf.seen.entrySet()) {
            int id = e.getKey();
            int idx = e.getValue();
            double lx = ekf.mu.get(3 + 2*idx);
            double ly = ekf.mu.get(3 + 2*idx + 1);
            System.out.println("Landmark " + id + " est: " + lx + " " + ly);
        }
    }
}
