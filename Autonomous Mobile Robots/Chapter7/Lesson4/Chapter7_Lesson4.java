/*
Chapter7_Lesson4.java
Kalman-Filter Localization for AMR — Lesson 4
EKF tuning via innovation statistics (NIS) for unicycle + GPS example.

Dependencies:
  EJML (Efficient Java Matrix Library)

Suggested Gradle dependency:
  implementation 'org.ejml:ejml-simple:0.43'
*/

import org.ejml.simple.SimpleMatrix;

import java.util.Random;

public class Chapter7_Lesson4 {

    static double wrapAngle(double a) {
        while (a > Math.PI)  a -= 2.0*Math.PI;
        while (a <= -Math.PI) a += 2.0*Math.PI;
        return a;
    }

    static class NoiseParams {
        double sigmaV;
        double sigmaW;
        double sigmaGps;
        double qScale = 1.0;
        double rScale = 1.0;
    }

    // x=[px,py,th], u=[v,w]
    static SimpleMatrix f(SimpleMatrix x, SimpleMatrix u, double dt) {
        double px = x.get(0), py = x.get(1), th = x.get(2);
        double v = u.get(0), w = u.get(1);
        SimpleMatrix x2 = new SimpleMatrix(3,1);
        x2.set(0, px + v*dt*Math.cos(th));
        x2.set(1, py + v*dt*Math.sin(th));
        x2.set(2, wrapAngle(th + w*dt));
        return x2;
    }

    static SimpleMatrix jacobianF(SimpleMatrix x, SimpleMatrix u, double dt) {
        double th = x.get(2);
        double v = u.get(0);
        SimpleMatrix F = SimpleMatrix.identity(3);
        F.set(0,2, -v*dt*Math.sin(th));
        F.set(1,2,  v*dt*Math.cos(th));
        return F;
    }

    static SimpleMatrix jacobianL(SimpleMatrix x, double dt) {
        double th = x.get(2);
        SimpleMatrix L = new SimpleMatrix(3,2);
        L.set(0,0, dt*Math.cos(th));
        L.set(1,0, dt*Math.sin(th));
        L.set(2,1, dt);
        return L;
    }

    static SimpleMatrix makeQ(SimpleMatrix x, SimpleMatrix u, double dt, NoiseParams p) {
        SimpleMatrix L = jacobianL(x, dt);
        SimpleMatrix M = new SimpleMatrix(2,2);
        M.set(0,0, p.sigmaV*p.sigmaV);
        M.set(1,1, p.sigmaW*p.sigmaW);
        // Q = qScale * L M L^T
        return L.mult(M).mult(L.transpose()).scale(p.qScale);
    }

    static SimpleMatrix hGps(SimpleMatrix x) {
        SimpleMatrix z = new SimpleMatrix(2,1);
        z.set(0, x.get(0));
        z.set(1, x.get(1));
        return z;
    }

    static SimpleMatrix jacobianH() {
        SimpleMatrix H = new SimpleMatrix(2,3);
        H.set(0,0, 1.0);
        H.set(1,1, 1.0);
        return H;
    }

    static SimpleMatrix makeR(NoiseParams p) {
        SimpleMatrix R = new SimpleMatrix(2,2);
        R.set(0,0, p.sigmaGps*p.sigmaGps);
        R.set(1,1, p.sigmaGps*p.sigmaGps);
        return R.scale(p.rScale);
    }

    static double nis(SimpleMatrix innov, SimpleMatrix S) {
        return innov.transpose().mult(S.invert()).mult(innov).get(0);
    }

    static class SimData {
        SimpleMatrix[] xTrue;
        SimpleMatrix[] uMeas;
        SimpleMatrix[] zGps;
    }

    static SimData simulate(double T, double dt, long seed) {
        int N = (int)Math.floor(T/dt);
        SimData d = new SimData();
        d.xTrue = new SimpleMatrix[N];
        d.uMeas = new SimpleMatrix[N];
        d.zGps  = new SimpleMatrix[N];

        for (int k=0; k<N; ++k) {
            d.xTrue[k] = new SimpleMatrix(3,1);
            d.uMeas[k] = new SimpleMatrix(2,1);
            d.zGps[k]  = new SimpleMatrix(2,1);
        }

        SimpleMatrix[] uTrue = new SimpleMatrix[N];
        for (int k=0; k<N; ++k) uTrue[k] = new SimpleMatrix(2,1);

        for (int k=0; k<N; ++k) {
            double t = k*dt;
            double v = 1.0 + 0.2*Math.sin(0.2*t);
            double w = 0.2*Math.sin(0.1*t);
            uTrue[k].set(0,v);
            uTrue[k].set(1,w);
            if (k > 0) d.xTrue[k] = f(d.xTrue[k-1], uTrue[k-1], dt);
        }

        Random rng = new Random(seed);
        double sigmaVMeas = 0.08;
        double sigmaWMeas = 0.04;
        double sigmaGpsMeas = 0.6;

        for (int k=0; k<N; ++k) {
            d.uMeas[k].set(0, uTrue[k].get(0) + sigmaVMeas*rng.nextGaussian());
            d.uMeas[k].set(1, uTrue[k].get(1) + sigmaWMeas*rng.nextGaussian());
            d.zGps[k].set(0, d.xTrue[k].get(0) + sigmaGpsMeas*rng.nextGaussian());
            d.zGps[k].set(1, d.xTrue[k].get(1) + sigmaGpsMeas*rng.nextGaussian());
        }
        return d;
    }

    static double mean(double[] a) {
        double s = 0.0;
        for (double v: a) s += v;
        return s / a.length;
    }

    public static void main(String[] args) {
        double dt = 0.1;
        SimData d = simulate(60.0, dt, 2);

        NoiseParams p = new NoiseParams();
        p.sigmaV = 0.03;
        p.sigmaW = 0.01;
        p.sigmaGps = 0.3;

        SimpleMatrix H = jacobianH();

        for (int it=0; it<4; ++it) {
            SimpleMatrix x = new SimpleMatrix(3,1); // start at 0
            SimpleMatrix P = new SimpleMatrix(3,3);
            P.set(0,0, 1.0);
            P.set(1,1, 1.0);
            double sTh = 10.0*Math.PI/180.0;
            P.set(2,2, sTh*sTh);

            int N = d.xTrue.length;
            double[] nisList = new double[N];

            for (int k=0; k<N; ++k) {
                // Predict
                SimpleMatrix xPred = f(x, d.uMeas[k], dt);
                SimpleMatrix F = jacobianF(x, d.uMeas[k], dt);
                SimpleMatrix Q = makeQ(x, d.uMeas[k], dt, p);
                SimpleMatrix PPred = F.mult(P).mult(F.transpose()).plus(Q);

                // Update
                SimpleMatrix y = d.zGps[k].minus(hGps(xPred));
                SimpleMatrix S = H.mult(PPred).mult(H.transpose()).plus(makeR(p));
                SimpleMatrix K = PPred.mult(H.transpose()).mult(S.invert());
                x = xPred.plus(K.mult(y));
                x.set(2, wrapAngle(x.get(2)));
                P = (SimpleMatrix.identity(3).minus(K.mult(H))).mult(PPred);

                nisList[k] = nis(y, S);
            }

            double nisMean = mean(nisList);
            System.out.println("Iter " + it + ": rScale=" + p.rScale + ", mean NIS=" + nisMean + " (target ~ 2)");
            double gain = Math.min(5.0, Math.max(0.2, nisMean/2.0));
            p.rScale *= gain;
        }

        System.out.println("Final rScale=" + p.rScale);
    }
}
