// Chapter10_Lesson4.java
// Robust scan matching (2D) under dynamic obstacles using IRLS-Huber + trimming
// Pure Java (no external deps). Educational O(NM) nearest neighbors.

import java.util.*;
import static java.lang.Math.*;

public class Chapter10_Lesson4 {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 add(Vec2 b) { return new Vec2(x + b.x, y + b.y); }
        Vec2 sub(Vec2 b) { return new Vec2(x - b.x, y - b.y); }
        Vec2 mul(double s) { return new Vec2(s * x, s * y); }
        double norm2() { return x*x + y*y; }
        double norm() { return sqrt(norm2()); }
    }

    static class Pose2 {
        double theta; // rotation angle
        Vec2 t;       // translation
        Pose2(double theta, Vec2 t) { this.theta = theta; this.t = t; }
    }

    static Vec2 rot(Vec2 p, double theta) {
        double c = cos(theta), s = sin(theta);
        return new Vec2(c*p.x - s*p.y, s*p.x + c*p.y);
    }

    static Vec2 transform(Vec2 p, Pose2 T) {
        Vec2 rp = rot(p, T.theta);
        return rp.add(T.t);
    }

    static double huberWeight(double r, double delta) {
        return (r <= delta) ? 1.0 : (delta / (r + 1e-12));
    }

    static int[] nearestNeighbors(List<Vec2> A, List<Vec2> B) {
        int[] idx = new int[A.size()];
        for (int i = 0; i < A.size(); ++i) {
            Vec2 a = A.get(i);
            double best = Double.POSITIVE_INFINITY;
            int bestj = -1;
            for (int j = 0; j < B.size(); ++j) {
                double dx = a.x - B.get(j).x;
                double dy = a.y - B.get(j).y;
                double dist2 = dx*dx + dy*dy;
                if (dist2 < best) { best = dist2; bestj = j; }
            }
            idx[i] = bestj;
        }
        return idx;
    }

    // Weighted 2D Kabsch in angle form:
    // theta = atan2(S12 - S21, S11 + S22) where S = sum w * (p-pbar)(q-qbar)^T
    static Pose2 weightedKabsch2D(List<Vec2> P, List<Vec2> Q, double[] w) {
        double wsum = 1e-12;
        double px = 0, py = 0, qx = 0, qy = 0;
        for (int i = 0; i < P.size(); ++i) {
            wsum += w[i];
            px += w[i] * P.get(i).x; py += w[i] * P.get(i).y;
            qx += w[i] * Q.get(i).x; qy += w[i] * Q.get(i).y;
        }
        px /= wsum; py /= wsum; qx /= wsum; qy /= wsum;

        double S11=0, S12=0, S21=0, S22=0;
        for (int i = 0; i < P.size(); ++i) {
            double x1 = P.get(i).x - px;
            double x2 = P.get(i).y - py;
            double y1 = Q.get(i).x - qx;
            double y2 = Q.get(i).y - qy;
            S11 += w[i] * x1 * y1;
            S12 += w[i] * x1 * y2;
            S21 += w[i] * x2 * y1;
            S22 += w[i] * x2 * y2;
        }

        double theta = atan2(S12 - S21, S11 + S22);
        Vec2 t = new Vec2(qx, qy).sub(rot(new Vec2(px, py), theta));
        return new Pose2(theta, t);
    }

    static Pose2 robustICP2D(List<Vec2> src, List<Vec2> tgt, int iters, double delta, double keepRatio) {
        Pose2 T = new Pose2(0.0, new Vec2(0.0, 0.0));

        for (int k = 0; k < iters; ++k) {
            ArrayList<Vec2> srcW = new ArrayList<>(src.size());
            for (Vec2 p : src) srcW.add(transform(p, T));

            int[] nn = nearestNeighbors(srcW, tgt);

            double[] r = new double[src.size()];
            for (int i = 0; i < src.size(); ++i) {
                Vec2 d = srcW.get(i).sub(tgt.get(nn[i]));
                r[i] = d.norm();
            }

            double[] rSorted = r.clone();
            Arrays.sort(rSorted);
            int qIdx = (int)floor(keepRatio * (rSorted.length - 1));
            double thr = rSorted[qIdx];

            ArrayList<Vec2> P_in = new ArrayList<>();
            ArrayList<Vec2> Q_in = new ArrayList<>();
            ArrayList<Double> wList = new ArrayList<>();

            for (int i = 0; i < src.size(); ++i) {
                if (r[i] <= thr) {
                    P_in.add(src.get(i));
                    Q_in.add(tgt.get(nn[i]));
                    wList.add(huberWeight(r[i], delta));
                }
            }

            double[] w_in = new double[wList.size()];
            for (int i = 0; i < w_in.length; ++i) w_in[i] = wList.get(i);

            Pose2 dT = weightedKabsch2D(P_in, Q_in, w_in);

            // Compose: dT o T
            double thetaNew = dT.theta + T.theta;
            Vec2 tNew = rot(T.t, dT.theta).add(dT.t);
            T = new Pose2(thetaNew, tNew);
        }

        return T;
    }

    static class Scene {
        ArrayList<Vec2> scan1, scan2;
        Pose2 Ttrue;
        Scene(ArrayList<Vec2> s1, ArrayList<Vec2> s2, Pose2 T) { scan1=s1; scan2=s2; Ttrue=T; }
    }

    static Scene makeSyntheticScene(int nStatic, int nDyn, long seed) {
        Random rng = new Random(seed);

        ArrayList<Vec2> stat = new ArrayList<>();
        for (int i = 0; i < nStatic/2; ++i) {
            double ang = 2.0 * PI * rng.nextDouble();
            stat.add(new Vec2(2.0*cos(ang), 2.0*sin(ang)));
        }
        for (int i = 0; i < nStatic/4; ++i) stat.add(new Vec2(-3.0 + 6.0*rng.nextDouble(), -1.5));
        for (int i = 0; i < nStatic/4; ++i) stat.add(new Vec2(1.5, -2.0 + 4.0*rng.nextDouble()));

        ArrayList<Vec2> dyn1 = new ArrayList<>();
        Vec2 c1 = new Vec2(-0.5, 0.8);
        for (int i = 0; i < nDyn; ++i) dyn1.add(c1.add(new Vec2(0.15*rng.nextGaussian(), 0.15*rng.nextGaussian())));

        double thetaTrue = toRadians(12.0);
        Vec2 tTrue = new Vec2(0.35, -0.10);
        Pose2 Ttrue = new Pose2(thetaTrue, tTrue);

        ArrayList<Vec2> scan1 = new ArrayList<>();
        scan1.addAll(stat);
        scan1.addAll(dyn1);

        for (int i = 0; i < scan1.size(); ++i) {
            Vec2 p = scan1.get(i);
            scan1.set(i, new Vec2(p.x + 0.02*rng.nextGaussian(), p.y + 0.02*rng.nextGaussian()));
        }

        ArrayList<Vec2> dyn2 = new ArrayList<>();
        Vec2 c2 = c1.add(new Vec2(0.55, -0.25));
        for (int i = 0; i < nDyn; ++i) dyn2.add(c2.add(new Vec2(0.15*rng.nextGaussian(), 0.15*rng.nextGaussian())));

        ArrayList<Vec2> scan2 = new ArrayList<>();
        for (Vec2 p : stat) scan2.add(transform(p, Ttrue));
        scan2.addAll(dyn2);

        for (int i = 0; i < scan2.size(); ++i) {
            Vec2 p = scan2.get(i);
            scan2.set(i, new Vec2(p.x + 0.02*rng.nextGaussian(), p.y + 0.02*rng.nextGaussian()));
        }

        return new Scene(scan1, scan2, Ttrue);
    }

    static double poseAngleErrorDeg(Pose2 Test, Pose2 Ttrue) {
        return abs(toDegrees(Test.theta - Ttrue.theta));
    }

    static double poseTranslationError(Pose2 Test, Pose2 Ttrue) {
        return Test.t.sub(Ttrue.t).norm();
    }

    public static void main(String[] args) {
        Scene sc = makeSyntheticScene(250, 60, 7);
        Pose2 Test = robustICP2D(sc.scan1, sc.scan2, 25, 0.18, 0.85);

        System.out.printf("True t = (%.3f, %.3f)%n", sc.Ttrue.t.x, sc.Ttrue.t.y);
        System.out.printf("Est  t = (%.3f, %.3f)%n", Test.t.x, Test.t.y);
        System.out.printf("Angle error (deg) = %.3f%n", poseAngleErrorDeg(Test, sc.Ttrue));
        System.out.printf("Translation error  = %.3f%n", poseTranslationError(Test, sc.Ttrue));
    }
}
