/*
Chapter7_Lesson2.java
UKF localization for planar (x, y, theta) robot with range-bearing landmarks.

Dependencies (recommended):
  - EJML (Efficient Java Matrix Library), e.g., org.ejml:ejml-simple

This example uses EJML's SimpleMatrix for readability. If you do not have EJML, you can
replace SimpleMatrix operations with your own small matrix utilities (n=3 here).
*/

import org.ejml.simple.SimpleMatrix;

import java.util.Random;

public class Chapter7_Lesson2 {

    static double wrapAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    static double meanAngle(double[] angles, double[] w) {
        double s = 0.0, c = 0.0;
        for (int i = 0; i < angles.length; i++) {
            s += w[i] * Math.sin(angles[i]);
            c += w[i] * Math.cos(angles[i]);
        }
        return wrapAngle(Math.atan2(s, c));
    }

    static class UKFParams {
        double alpha = 0.35;
        double beta  = 2.0;
        double kappa = 0.0;
    }

    static class UKF {
        final int n = 3;
        final UKFParams params;
        final double lambda;

        final double[] Wm;
        final double[] Wc;

        SimpleMatrix x; // 3x1
        SimpleMatrix P; // 3x3
        SimpleMatrix Q; // 3x3

        UKF(UKFParams p) {
            this.params = p;
            this.lambda = p.alpha * p.alpha * (n + p.kappa) - n;

            Wm = new double[2 * n + 1];
            Wc = new double[2 * n + 1];

            double w = 1.0 / (2.0 * (n + lambda));
            for (int i = 0; i < Wm.length; i++) {
                Wm[i] = w;
                Wc[i] = w;
            }
            Wm[0] = lambda / (n + lambda);
            Wc[0] = Wm[0] + (1.0 - p.alpha * p.alpha + p.beta);

            x = new SimpleMatrix(n, 1);
            P = SimpleMatrix.identity(n);
            Q = SimpleMatrix.identity(n).scale(1e-3);
        }

        void setState(double px, double py, double th, SimpleMatrix P0) {
            x.set(0, 0, px);
            x.set(1, 0, py);
            x.set(2, 0, wrapAngle(th));
            P = P0.copy();
        }

        void setProcessNoise(SimpleMatrix Q0) {
            Q = Q0.copy();
        }

        // sigma points matrix: (2n+1) x n
        SimpleMatrix sigmaPoints(SimpleMatrix x, SimpleMatrix P) {
            SimpleMatrix A = P.scale(n + lambda);
            // EJML: chol() returns a Cholesky decomposition object; SimpleMatrix.chol().getT() returns a triangular factor.
            // For most EJML versions, A = T^T * T, with T upper-triangular. We use S = T^T (lower-triangular).
            SimpleMatrix T = A.chol().getT(null); // triangular factor
            SimpleMatrix S = T.transpose();

            SimpleMatrix X = new SimpleMatrix(2 * n + 1, n);
            for (int j = 0; j < n; j++) X.set(0, j, x.get(j, 0));

            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    double sji = S.get(j, i);
                    X.set(i + 1,     j, x.get(j, 0) + sji);
                    X.set(i + 1 + n, j, x.get(j, 0) - sji);
                }
            }

            // wrap theta
            for (int r = 0; r < X.numRows(); r++) {
                X.set(r, 2, wrapAngle(X.get(r, 2)));
            }
            return X;
        }

        SimpleMatrix stateMean(SimpleMatrix X) {
            double mx = 0.0, my = 0.0;
            double[] th = new double[X.numRows()];
            for (int i = 0; i < X.numRows(); i++) {
                mx += Wm[i] * X.get(i, 0);
                my += Wm[i] * X.get(i, 1);
                th[i] = X.get(i, 2);
            }
            double mth = meanAngle(th, Wm);
            SimpleMatrix m = new SimpleMatrix(n, 1);
            m.set(0, 0, mx);
            m.set(1, 0, my);
            m.set(2, 0, mth);
            return m;
        }

        SimpleMatrix stateResidual(SimpleMatrix a, SimpleMatrix b) {
            SimpleMatrix y = a.minus(b);
            y.set(2, 0, wrapAngle(y.get(2, 0)));
            return y;
        }

        interface MotionFcn {
            SimpleMatrix f(SimpleMatrix x, SimpleMatrix u);
        }

        interface MeasFcn {
            SimpleMatrix h(SimpleMatrix x);
        }

        void predict(MotionFcn f, SimpleMatrix u) {
            SimpleMatrix X = sigmaPoints(x, P);
            SimpleMatrix Xp = new SimpleMatrix(X.numRows(), X.numCols());

            for (int i = 0; i < X.numRows(); i++) {
                SimpleMatrix xi = X.extractVector(true, i).transpose(); // n x 1
                SimpleMatrix yi = f.f(xi, u);
                yi.set(2, 0, wrapAngle(yi.get(2, 0)));
                for (int j = 0; j < n; j++) Xp.set(i, j, yi.get(j, 0));
            }

            SimpleMatrix xPred = stateMean(Xp);

            SimpleMatrix PPred = new SimpleMatrix(n, n);
            for (int i = 0; i < Xp.numRows(); i++) {
                SimpleMatrix xi = Xp.extractVector(true, i).transpose();
                SimpleMatrix dx = stateResidual(xi, xPred);
                PPred = PPred.plus(dx.mult(dx.transpose()).scale(Wc[i]));
            }
            PPred = PPred.plus(Q);

            x = xPred;
            P = PPred;
        }

        void updateSequential(SimpleMatrix z, MeasFcn h, SimpleMatrix R) {
            SimpleMatrix X = sigmaPoints(x, P);

            // propagate sigma points to measurement space (2D measurement: [range, bearing])
            SimpleMatrix Z = new SimpleMatrix(X.numRows(), 2);
            for (int i = 0; i < X.numRows(); i++) {
                SimpleMatrix xi = X.extractVector(true, i).transpose();
                SimpleMatrix zi = h.h(xi);
                zi.set(1, 0, wrapAngle(zi.get(1, 0)));
                Z.set(i, 0, zi.get(0, 0));
                Z.set(i, 1, zi.get(1, 0));
            }

            double mz0 = 0.0;
            double[] bears = new double[Z.numRows()];
            for (int i = 0; i < Z.numRows(); i++) {
                mz0 += Wm[i] * Z.get(i, 0);
                bears[i] = Z.get(i, 1);
            }
            double mz1 = meanAngle(bears, Wm);

            SimpleMatrix zPred = new SimpleMatrix(2, 1);
            zPred.set(0, 0, mz0);
            zPred.set(1, 0, mz1);

            SimpleMatrix S = new SimpleMatrix(2, 2);
            SimpleMatrix Pxz = new SimpleMatrix(n, 2);

            for (int i = 0; i < Z.numRows(); i++) {
                SimpleMatrix zi = new SimpleMatrix(2, 1);
                zi.set(0, 0, Z.get(i, 0));
                zi.set(1, 0, Z.get(i, 1));

                SimpleMatrix dz = zi.minus(zPred);
                dz.set(1, 0, wrapAngle(dz.get(1, 0)));

                SimpleMatrix xi = X.extractVector(true, i).transpose();
                SimpleMatrix dx = stateResidual(xi, x);

                S = S.plus(dz.mult(dz.transpose()).scale(Wc[i]));
                Pxz = Pxz.plus(dx.mult(dz.transpose()).scale(Wc[i]));
            }
            S = S.plus(R);

            SimpleMatrix K = Pxz.mult(S.invert());
            SimpleMatrix innov = z.minus(zPred);
            innov.set(1, 0, wrapAngle(innov.get(1, 0)));

            x = x.plus(K.mult(innov));
            x.set(2, 0, wrapAngle(x.get(2, 0)));
            P = P.minus(K.mult(S).mult(K.transpose()));
        }
    }

    static SimpleMatrix motionModel(SimpleMatrix x, SimpleMatrix u, double dt) {
        double px = x.get(0, 0);
        double py = x.get(1, 0);
        double th = x.get(2, 0);
        double v  = u.get(0, 0);
        double w  = u.get(1, 0);

        SimpleMatrix y = new SimpleMatrix(3, 1);
        y.set(0, 0, px + dt * v * Math.cos(th));
        y.set(1, 0, py + dt * v * Math.sin(th));
        y.set(2, 0, wrapAngle(th + dt * w));
        return y;
    }

    static SimpleMatrix measModel(SimpleMatrix x, double lx, double ly) {
        double px = x.get(0, 0);
        double py = x.get(1, 0);
        double th = x.get(2, 0);

        double dx = lx - px;
        double dy = ly - py;
        double r = Math.sqrt(dx * dx + dy * dy);
        double b = wrapAngle(Math.atan2(dy, dx) - th);

        SimpleMatrix z = new SimpleMatrix(2, 1);
        z.set(0, 0, r);
        z.set(1, 0, b);
        return z;
    }

    public static void main(String[] args) {
        Random rng = new Random(7);

        double dt = 0.1;
        int T = 250;

        double[][] landmarks = new double[][]{
                {5.0, 0.0},
                {0.0, 6.0},
                {6.0, 6.0},
                {8.0, -2.0}
        };

        SimpleMatrix xTrue = new SimpleMatrix(3, 1);
        xTrue.set(0, 0, 0.0);
        xTrue.set(1, 0, 0.0);
        xTrue.set(2, 0, 0.2);

        UKF ukf = new UKF(new UKFParams());
        SimpleMatrix P0 = new SimpleMatrix(3, 3);
        P0.set(0, 0, 0.8 * 0.8);
        P0.set(1, 1, 0.8 * 0.8);
        P0.set(2, 2, Math.pow(20.0 * Math.PI / 180.0, 2.0));
        ukf.setState(0.5, -0.5, -0.3, P0);

        double sigmaXY = 0.02;
        double sigmaTh = 1.0 * Math.PI / 180.0;
        SimpleMatrix Q = new SimpleMatrix(3, 3);
        Q.set(0, 0, sigmaXY * sigmaXY);
        Q.set(1, 1, sigmaXY * sigmaXY);
        Q.set(2, 2, sigmaTh * sigmaTh);
        ukf.setProcessNoise(Q);

        double sigmaR = 0.15;
        double sigmaB = 2.0 * Math.PI / 180.0;
        SimpleMatrix R = new SimpleMatrix(2, 2);
        R.set(0, 0, sigmaR * sigmaR);
        R.set(1, 1, sigmaB * sigmaB);

        for (int k = 0; k < T; k++) {
            double v = 1.0 + 0.2 * Math.sin(0.07 * k);
            double w = 0.35 * Math.sin(0.03 * k);
            SimpleMatrix u = new SimpleMatrix(2, 1);
            u.set(0, 0, v);
            u.set(1, 0, w);

            // true propagation with noise
            xTrue = motionModel(xTrue, u, dt);
            xTrue.set(0, 0, xTrue.get(0, 0) + sigmaXY * rng.nextGaussian());
            xTrue.set(1, 0, xTrue.get(1, 0) + sigmaXY * rng.nextGaussian());
            xTrue.set(2, 0, wrapAngle(xTrue.get(2, 0) + sigmaTh * rng.nextGaussian()));

            // UKF predict
            ukf.predict((xx, uu) -> motionModel(xx, uu, dt), u);

            // sequential landmark updates
            for (double[] lm : landmarks) {
                double lx = lm[0], ly = lm[1];
                SimpleMatrix z = measModel(xTrue, lx, ly);
                z.set(0, 0, z.get(0, 0) + sigmaR * rng.nextGaussian());
                z.set(1, 0, wrapAngle(z.get(1, 0) + sigmaB * rng.nextGaussian()));

                final double flx = lx, fly = ly;
                ukf.updateSequential(z, (xx) -> measModel(xx, flx, fly), R);
            }
        }

        System.out.println("Final true state: " + xTrue.transpose());
        System.out.println("Final UKF  state: " + ukf.x.transpose());
        System.out.println("Final UKF covariance diag: " + ukf.P.extractDiag().transpose());
    }
}
