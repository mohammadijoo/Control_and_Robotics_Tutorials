/*
Chapter 11 - SLAM I (Filter-Based SLAM)
Lesson 2: EKF-SLAM (structure and limitations)

Minimal EKF-SLAM skeleton in Java using EJML for matrices.

Maven dependency (pom.xml):
  <dependency>
    <groupId>org.ejml</groupId>
    <artifactId>ejml-simple</artifactId>
    <version>0.43</version>
  </dependency>

Assumptions:
- 2D pose (x,y,theta)
- range-bearing measurements to point landmarks
- known data association
*/

import org.ejml.simple.SimpleMatrix;

import java.util.HashMap;
import java.util.Map;

public class Chapter11_Lesson2 {

    static double wrapAngle(double a) {
        a = (a + Math.PI) % (2.0 * Math.PI);
        if (a < 0) a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    static class Measurement {
        int id;
        double range;
        double bearing;
        Measurement(int id, double range, double bearing) {
            this.id = id;
            this.range = range;
            this.bearing = bearing;
        }
    }

    static class EKFSLAM {
        SimpleMatrix mu;  // (D x 1)
        SimpleMatrix P;   // (D x D)
        Map<Integer,Integer> idToSlot = new HashMap<>();

        EKFSLAM() {
            mu = new SimpleMatrix(3,1);
            P  = SimpleMatrix.identity(3).scale(1e-6);
        }

        int numLandmarks() { return idToSlot.size(); }
        int dim() { return 3 + 2 * numLandmarks(); }

        int lmIndex(int id) {
            int slot = idToSlot.get(id);
            return 3 + 2 * slot;
        }

        void predict(double v, double w, double dt, SimpleMatrix Q) {
            double x = mu.get(0), y = mu.get(1), th = mu.get(2);

            double xNew, yNew, thNew;
            if (Math.abs(w) < 1e-9) {
                xNew = x + v * dt * Math.cos(th);
                yNew = y + v * dt * Math.sin(th);
                thNew = th;
            } else {
                xNew = x + (v / w) * (Math.sin(th + w*dt) - Math.sin(th));
                yNew = y - (v / w) * (Math.cos(th + w*dt) - Math.cos(th));
                thNew = th + w * dt;
            }
            thNew = wrapAngle(thNew);

            mu.set(0, xNew);
            mu.set(1, yNew);
            mu.set(2, thNew);

            int D = dim();
            SimpleMatrix F = SimpleMatrix.identity(D);
            SimpleMatrix Gu = new SimpleMatrix(D, 2);

            if (Math.abs(w) < 1e-9) {
                F.set(0,2, -v*dt*Math.sin(th));
                F.set(1,2,  v*dt*Math.cos(th));

                Gu.set(0,0, dt*Math.cos(th));
                Gu.set(1,0, dt*Math.sin(th));
                Gu.set(2,1, dt);
            } else {
                F.set(0,2, (v/w)*(Math.cos(th + w*dt) - Math.cos(th)));
                F.set(1,2, (v/w)*(Math.sin(th + w*dt) - Math.sin(th)));

                Gu.set(0,0, (1.0/w)*(Math.sin(th + w*dt) - Math.sin(th)));
                Gu.set(1,0, -(1.0/w)*(Math.cos(th + w*dt) - Math.cos(th)));
                Gu.set(2,1, dt);

                Gu.set(0,1, (v/(w*w))*(Math.sin(th) - Math.sin(th + w*dt)) + (v/w)*(dt*Math.cos(th + w*dt)));
                Gu.set(1,1, (v/(w*w))*(Math.cos(th + w*dt) - Math.cos(th)) + (v/w)*(dt*Math.sin(th + w*dt)));
            }

            // P = F P F^T + Gu Q Gu^T
            SimpleMatrix Pnew = F.mult(P).mult(F.transpose()).plus(Gu.mult(Q).mult(Gu.transpose()));
            P = Pnew.plus(Pnew.transpose()).scale(0.5);
        }

        void update(Measurement m, SimpleMatrix R) {
            if (!idToSlot.containsKey(m.id)) {
                initializeLandmark(m, R);
            }

            SimpleMatrix z = new SimpleMatrix(2,1);
            z.set(0, m.range);
            z.set(1, wrapAngle(m.bearing));

            ExpectedJac ej = expectedAndJacobian(m.id);
            SimpleMatrix zhat = ej.zhat;
            SimpleMatrix H = ej.H;

            SimpleMatrix innov = z.minus(zhat);
            innov.set(1, wrapAngle(innov.get(1)));

            SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
            SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());

            mu = mu.plus(K.mult(innov));
            mu.set(2, wrapAngle(mu.get(2)));

            int D = dim();
            SimpleMatrix I = SimpleMatrix.identity(D);
            // Joseph form: P = (I-KH)P(I-KH)^T + K R K^T
            SimpleMatrix IKH = I.minus(K.mult(H));
            P = IKH.mult(P).mult(IKH.transpose()).plus(K.mult(R).mult(K.transpose()));
            P = P.plus(P.transpose()).scale(0.5);
        }

        static class ExpectedJac {
            SimpleMatrix zhat;
            SimpleMatrix H;
            ExpectedJac(SimpleMatrix zhat, SimpleMatrix H) { this.zhat = zhat; this.H = H; }
        }

        ExpectedJac expectedAndJacobian(int id) {
            int D = dim();
            SimpleMatrix H = new SimpleMatrix(2, D);

            double x = mu.get(0), y = mu.get(1), th = mu.get(2);
            int idx = lmIndex(id);
            double mx = mu.get(idx), my = mu.get(idx+1);

            double dx = mx - x;
            double dy = my - y;
            double q  = dx*dx + dy*dy;
            double r  = Math.sqrt(q);

            SimpleMatrix zhat = new SimpleMatrix(2,1);
            zhat.set(0, r);
            zhat.set(1, wrapAngle(Math.atan2(dy, dx) - th));

            // range
            H.set(0,0, -dx/r); H.set(0,1, -dy/r);
            H.set(0,idx, dx/r); H.set(0,idx+1, dy/r);

            // bearing
            H.set(1,0, dy/q);  H.set(1,1, -dx/q); H.set(1,2, -1.0);
            H.set(1,idx, -dy/q); H.set(1,idx+1, dx/q);

            return new ExpectedJac(zhat, H);
        }

        void initializeLandmark(Measurement m, SimpleMatrix R) {
            double x = mu.get(0), y = mu.get(1), th = mu.get(2);
            double r = m.range, b = m.bearing;
            double ang = th + b;

            double mx = x + r*Math.cos(ang);
            double my = y + r*Math.sin(ang);

            // Gx: 2x3
            SimpleMatrix Gx = new SimpleMatrix(2,3);
            Gx.set(0,0, 1.0); Gx.set(0,2, -r*Math.sin(ang));
            Gx.set(1,1, 1.0); Gx.set(1,2,  r*Math.cos(ang));

            // Gz: 2x2
            SimpleMatrix Gz = new SimpleMatrix(2,2);
            Gz.set(0,0, Math.cos(ang)); Gz.set(0,1, -r*Math.sin(ang));
            Gz.set(1,0, Math.sin(ang)); Gz.set(1,1,  r*Math.cos(ang));

            int D = dim();
            SimpleMatrix Prr = P.extractMatrix(0,3,0,3);
            SimpleMatrix Pxr = P.extractMatrix(0,D,0,3);

            SimpleMatrix Pmm = Gx.mult(Prr).mult(Gx.transpose()).plus(Gz.mult(R).mult(Gz.transpose()));
            SimpleMatrix Pxm = Pxr.mult(Gx.transpose()); // Dx2

            // Augment mu
            SimpleMatrix muNew = new SimpleMatrix(D+2,1);
            muNew.insertIntoThis(0,0, mu);
            muNew.set(D, mx);
            muNew.set(D+1, my);
            mu = muNew;

            // Augment P
            SimpleMatrix Pnew = new SimpleMatrix(D+2, D+2);
            Pnew.insertIntoThis(0,0, P);
            Pnew.insertIntoThis(0,D, Pxm);
            Pnew.insertIntoThis(D,0, Pxm.transpose());
            Pnew.insertIntoThis(D,D, Pmm);
            P = Pnew.plus(Pnew.transpose()).scale(0.5);

            idToSlot.put(m.id, numLandmarks()); // new last slot
        }
    }

    public static void main(String[] args) {
        EKFSLAM slam = new EKFSLAM();

        SimpleMatrix Q = new SimpleMatrix(new double[][]{
                {0.05*0.05, 0},
                {0, Math.toRadians(2.0)*Math.toRadians(2.0)}
        });
        SimpleMatrix R = new SimpleMatrix(new double[][]{
                {0.15*0.15, 0},
                {0, Math.toRadians(3.0)*Math.toRadians(3.0)}
        });

        for (int k = 0; k < 50; k++) {
            slam.predict(0.7, 0.1, 0.1, Q);
            slam.update(new Measurement(0, 5.0, 0.3), R);
        }

        System.out.println("State dim: " + slam.dim());
        System.out.println("Mean:\n" + slam.mu);
    }
}
