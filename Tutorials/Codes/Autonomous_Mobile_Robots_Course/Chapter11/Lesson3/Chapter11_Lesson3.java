// Chapter11_Lesson3.java
/*
FastSLAM 1.0 (Rao–Blackwellized PF-SLAM) — compact Java reference implementation.

Educational notes:
- 2D pose: (x,y,theta)
- Known landmark IDs in observations
- Each particle maintains EKF (mean+cov) per landmark
- Uses EJML for matrices (recommended for robotics/estimation in Java)

Gradle/Maven dependency (EJML):
  org.ejml:ejml-simple:0.43 (or similar)

This file focuses on algorithmic structure. Hook it to your simulator/sensors.
*/

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter11_Lesson3 {

    static double wrapAngle(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    static class LandmarkEKF {
        boolean seen = false;
        SimpleMatrix mu = new SimpleMatrix(2, 1);               // [mx; my]
        SimpleMatrix Sigma = SimpleMatrix.identity(2).scale(1e6);
    }

    static class Particle {
        double x, y, th;
        double w;
        LandmarkEKF[] lms;

        Particle(int M) {
            x = 0; y = 0; th = 0;
            w = 1.0;
            lms = new LandmarkEKF[M];
            for (int i = 0; i < M; i++) lms[i] = new LandmarkEKF();
        }

        Particle deepCopy() {
            Particle p = new Particle(lms.length);
            p.x = x; p.y = y; p.th = th;
            p.w = w;
            for (int i = 0; i < lms.length; i++) {
                p.lms[i].seen = lms[i].seen;
                p.lms[i].mu = lms[i].mu.copy();
                p.lms[i].Sigma = lms[i].Sigma.copy();
            }
            return p;
        }
    }

    static class Obs {
        int id;
        double range;
        double bearing;
        Obs(int id, double range, double bearing) {
            this.id = id;
            this.range = range;
            this.bearing = bearing;
        }
    }

    static double[] motionModel(double[] x, double v, double w, double dt) {
        double px = x[0], py = x[1], th = x[2];
        double px2, py2, th2;

        if (Math.abs(w) < 1e-9) {
            px2 = px + v * dt * Math.cos(th);
            py2 = py + v * dt * Math.sin(th);
            th2 = th;
        } else {
            px2 = px + (v / w) * (Math.sin(th + w * dt) - Math.sin(th));
            py2 = py - (v / w) * (Math.cos(th + w * dt) - Math.cos(th));
            th2 = th + w * dt;
        }
        return new double[]{px2, py2, wrapAngle(th2)};
    }

    static double[] invMeasurement(double[] x, double range, double bearing) {
        double px = x[0], py = x[1], th = x[2];
        double ang = th + bearing;
        return new double[]{px + range * Math.cos(ang), py + range * Math.sin(ang)};
    }

    static double[] hLandmark(double[] x, double[] m) {
        double px = x[0], py = x[1], th = x[2];
        double dx = m[0] - px;
        double dy = m[1] - py;
        double q = dx * dx + dy * dy;
        double r = Math.sqrt(Math.max(q, 1e-12));
        double b = wrapAngle(Math.atan2(dy, dx) - th);
        return new double[]{r, b};
    }

    static SimpleMatrix H_landmark(double[] x, double[] m) {
        double px = x[0], py = x[1];
        double dx = m[0] - px;
        double dy = m[1] - py;
        double q = Math.max(dx * dx + dy * dy, 1e-12);
        double r = Math.sqrt(Math.max(q, 1e-12));

        // dh/dm
        return new SimpleMatrix(new double[][]{
                {dx / r, dy / r},
                {-dy / q, dx / q}
        });
    }

    static double gaussLikelihood(SimpleMatrix v, SimpleMatrix S) {
        // 2D Gaussian N(v;0,S)
        SimpleMatrix Ssym = S.plus(S.transpose()).scale(0.5);
        double detS = Math.max(Ssym.determinant(), 1e-18);
        SimpleMatrix invS = Ssym.invert();
        double e = v.transpose().mult(invS).mult(v).get(0, 0);
        double norm = 1.0 / (2.0 * Math.PI * Math.sqrt(detS));
        return norm * Math.exp(-0.5 * e);
    }

    static void normalizeWeights(List<Particle> ps) {
        double sum = 0.0;
        for (Particle p : ps) sum += p.w;
        if (sum < 1e-30) {
            double w0 = 1.0 / ps.size();
            for (Particle p : ps) p.w = w0;
        } else {
            for (Particle p : ps) p.w /= sum;
        }
    }

    static double effectiveSampleSize(List<Particle> ps) {
        double s2 = 0.0;
        for (Particle p : ps) s2 += p.w * p.w;
        return (s2 > 1e-18) ? 1.0 / s2 : 0.0;
    }

    static List<Particle> systematicResample(List<Particle> ps, Random rng) {
        int N = ps.size();
        double[] cdf = new double[N];
        cdf[0] = ps.get(0).w;
        for (int i = 1; i < N; i++) cdf[i] = cdf[i - 1] + ps.get(i).w;

        double u0 = rng.nextDouble() / N;
        List<Particle> out = new ArrayList<>(N);

        int j = 0;
        for (int i = 0; i < N; i++) {
            double u = u0 + (double) i / N;
            while (j < N - 1 && u > cdf[j]) j++;
            Particle pnew = ps.get(j).deepCopy();
            pnew.w = 1.0 / N;
            out.add(pnew);
        }
        return out;
    }

    static void fastslamStep(List<Particle> ps,
                             double v, double w, double dt,
                             List<Obs> obs,
                             SimpleMatrix R_u, SimpleMatrix Q_z,
                             Random rng) {
        int N = ps.size();

        // 1) sample motion by perturbing controls (v,w)
        for (Particle p : ps) {
            double dv = rng.nextGaussian() * Math.sqrt(R_u.get(0, 0));
            double dw = rng.nextGaussian() * Math.sqrt(R_u.get(1, 1));
            double[] x2 = motionModel(new double[]{p.x, p.y, p.th}, v + dv, w + dw, dt);
            p.x = x2[0]; p.y = x2[1]; p.th = x2[2];
        }

        // 2) update landmark EKFs and weights
        for (Particle p : ps) {
            for (Obs o : obs) {
                LandmarkEKF lm = p.lms[o.id];

                double[] x = new double[]{p.x, p.y, p.th};
                double[] z = new double[]{o.range, o.bearing};

                if (!lm.seen) {
                    double[] m0 = invMeasurement(x, o.range, o.bearing);
                    lm.mu.set(0, 0, m0[0]);
                    lm.mu.set(1, 0, m0[1]);

                    SimpleMatrix H = H_landmark(x, m0);
                    SimpleMatrix J = H.invert();
                    lm.Sigma = J.mult(Q_z).mult(J.transpose());
                    lm.seen = true;
                    continue;
                }

                double[] muArr = new double[]{lm.mu.get(0, 0), lm.mu.get(1, 0)};
                double[] zhat = hLandmark(x, muArr);

                double vr = z[0] - zhat[0];
                double vb = wrapAngle(z[1] - zhat[1]);
                SimpleMatrix vvec = new SimpleMatrix(new double[][]{{vr}, {vb}});

                SimpleMatrix H = H_landmark(x, muArr);
                SimpleMatrix S = H.mult(lm.Sigma).mult(H.transpose()).plus(Q_z);
                SimpleMatrix K = lm.Sigma.mult(H.transpose()).mult(S.invert());

                lm.mu = lm.mu.plus(K.mult(vvec));
                lm.Sigma = SimpleMatrix.identity(2).minus(K.mult(H)).mult(lm.Sigma);

                p.w *= gaussLikelihood(vvec, S);
            }
        }

        normalizeWeights(ps);

        double ess = effectiveSampleSize(ps);
        if (ess < 0.5 * N) {
            List<Particle> res = systematicResample(ps, rng);
            ps.clear();
            ps.addAll(res);
        }
    }

    public static void main(String[] args) {
        int M = 5;    // number of landmarks
        int N = 200;  // particles
        int T = 20;
        double dt = 0.1;

        // Noise covariances
        SimpleMatrix R_u = SimpleMatrix.identity(2);
        R_u.set(0, 0, 0.05 * 0.05);
        R_u.set(1, 1, Math.pow(2.0 * Math.PI / 180.0, 2));

        SimpleMatrix Q_z = SimpleMatrix.identity(2);
        Q_z.set(0, 0, 0.15 * 0.15);
        Q_z.set(1, 1, Math.pow(3.0 * Math.PI / 180.0, 2));

        List<Particle> ps = new ArrayList<>(N);
        for (int i = 0; i < N; i++) {
            Particle p = new Particle(M);
            p.w = 1.0 / N;
            ps.add(p);
        }

        Random rng = new Random(42);

        for (int t = 0; t < T; t++) {
            double v = 0.8;
            double w = 0.1 * Math.sin(0.1 * t);

            // Demo uses empty observations. Hook your simulator here.
            List<Obs> obs = new ArrayList<>();

            fastslamStep(ps, v, w, dt, obs, R_u, Q_z, rng);

            // Weighted mean pose
            double mx = 0, my = 0, c = 0, s = 0;
            for (Particle p : ps) {
                mx += p.w * p.x;
                my += p.w * p.y;
                c += p.w * Math.cos(p.th);
                s += p.w * Math.sin(p.th);
            }
            double mth = Math.atan2(s, c);

            System.out.printf("t=%d mean pose: (%.3f, %.3f, %.3f)%n", t, mx, my, mth);
        }
    }
}
