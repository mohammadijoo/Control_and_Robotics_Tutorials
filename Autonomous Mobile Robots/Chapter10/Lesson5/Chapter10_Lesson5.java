// Chapter10_Lesson5.java
/*
Autonomous Mobile Robots — Chapter 10, Lesson 5
Lab: ICP-Based Motion Estimation (2D)

A dependency-free 2D ICP example (no external linear algebra needed).
- nearest-neighbor correspondences via brute force
- closed-form 2D least-squares rotation:
    a = sum( x·y ), b = sum( x cross y ), theta = atan2(b, a)

Compile:
  javac Chapter10_Lesson5.java
Run:
  java Chapter10_Lesson5
*/

import java.util.*;
import static java.lang.Math.*;

public class Chapter10_Lesson5 {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 add(Vec2 o){ return new Vec2(x+o.x, y+o.y); }
        Vec2 sub(Vec2 o){ return new Vec2(x-o.x, y-o.y); }
        Vec2 mul(double s){ return new Vec2(s*x, s*y); }
        double dot(Vec2 o){ return x*o.x + y*o.y; }
        double cross(Vec2 o){ return x*o.y - y*o.x; } // 2D scalar cross
        double norm(){ return sqrt(x*x + y*y); }
    }

    static class Mat2 {
        double a,b,c,d; // [[a,b],[c,d]]
        Mat2(double a,double b,double c,double d){ this.a=a; this.b=b; this.c=c; this.d=d; }
        static Mat2 rot(double th){
            double cs = cos(th), sn = sin(th);
            return new Mat2(cs, -sn, sn, cs);
        }
        Vec2 mul(Vec2 v){ return new Vec2(a*v.x + b*v.y, c*v.x + d*v.y); }
        Mat2 mul(Mat2 o){
            return new Mat2(
                a*o.a + b*o.c, a*o.b + b*o.d,
                c*o.a + d*o.c, c*o.b + d*o.d
            );
        }
        Mat2 T(){ return new Mat2(a,c,b,d); }
    }

    static Vec2 centroid(List<Vec2> P){
        Vec2 s = new Vec2(0,0);
        for (Vec2 p: P) s = s.add(p);
        return s.mul(1.0 / max(1, P.size()));
    }

    static List<Vec2> apply(List<Vec2> P, Mat2 R, Vec2 t){
        ArrayList<Vec2> out = new ArrayList<>(P.size());
        for (Vec2 p: P) out.add(R.mul(p).add(t));
        return out;
    }

    static int nnIndex(Vec2 p, List<Vec2> Q){
        int best = 0;
        double bestd = Double.POSITIVE_INFINITY;
        for (int j=0;j<Q.size();j++){
            Vec2 d = Q.get(j).sub(p);
            double dd = d.x*d.x + d.y*d.y;
            if (dd < bestd){ bestd = dd; best = j; }
        }
        return best;
    }

    static void bestFit2D(List<Vec2> src, List<Vec2> dst, Mat2[] Rout, Vec2[] tout){
        Vec2 muS = centroid(src);
        Vec2 muD = centroid(dst);

        double A = 0.0, B = 0.0;
        for (int i=0;i<src.size();i++){
            Vec2 xs = src.get(i).sub(muS);
            Vec2 yd = dst.get(i).sub(muD);
            A += xs.dot(yd);
            B += xs.cross(yd);
        }
        double theta = atan2(B, A);
        Mat2 R = Mat2.rot(theta);
        Vec2 t = muD.sub(R.mul(muS));

        Rout[0] = R;
        tout[0] = t;
    }

    static class ICPResult {
        Mat2 R; Vec2 t; int iters; double rmse;
        ICPResult(Mat2 R, Vec2 t, int iters, double rmse){
            this.R=R; this.t=t; this.iters=iters; this.rmse=rmse;
        }
    }

    static ICPResult icp2D(List<Vec2> src0, List<Vec2> dst0,
                           Mat2 R, Vec2 t,
                           int maxIter, double tol,
                           double rejectDist, double trimFraction){
        double prev = Double.POSITIVE_INFINITY;

        for (int it=0; it<maxIter; it++){
            List<Vec2> srcT = apply(src0, R, t);

            ArrayList<Vec2> srcM = new ArrayList<>();
            ArrayList<Vec2> dstM = new ArrayList<>();
            ArrayList<Double> dist = new ArrayList<>();

            for (int i=0;i<srcT.size();i++){
                Vec2 p = srcT.get(i);
                int j = nnIndex(p, dst0);
                double d = dst0.get(j).sub(p).norm();
                if (d <= rejectDist){
                    srcM.add(p);
                    dstM.add(dst0.get(j));
                    dist.add(d);
                }
            }
            if (srcM.size() < 3) throw new RuntimeException("Too few correspondences after gating.");

            // trimming
            Integer[] idx = new Integer[srcM.size()];
            for (int i=0;i<idx.length;i++) idx[i]=i;
            Arrays.sort(idx, Comparator.comparingDouble(i -> dist.get(i)));
            int k = max(3, (int)(trimFraction * idx.length));

            ArrayList<Vec2> srcK = new ArrayList<>(k);
            ArrayList<Vec2> dstK = new ArrayList<>(k);
            for (int ii=0; ii<k; ii++){
                int i = idx[ii];
                srcK.add(srcM.get(i));
                dstK.add(dstM.get(i));
            }

            Mat2[] dR = new Mat2[1];
            Vec2[] dt = new Vec2[1];
            bestFit2D(srcK, dstK, dR, dt);

            // compose update
            R = dR[0].mul(R);
            t = dR[0].mul(t).add(dt[0]);

            // RMSE on trimmed set after update (approx)
            double sse = 0.0;
            for (int i=0;i<k;i++){
                Vec2 e = dR[0].mul(srcK.get(i)).add(dt[0]).sub(dstK.get(i));
                sse += e.x*e.x + e.y*e.y;
            }
            double rmse = sqrt(sse / max(1, k));

            if (abs(prev - rmse) < tol) return new ICPResult(R, t, it+1, rmse);
            prev = rmse;
        }
        return new ICPResult(R, t, maxIter, prev);
    }

    static List<Vec2> simulateWorld(int n, Random rng){
        ArrayList<Vec2> P = new ArrayList<>(n);
        for (int i=0;i<n/4;i++) P.add(new Vec2(rng.nextDouble()*8.0, 0.0));
        for (int i=0;i<n/4;i++) P.add(new Vec2(0.0, rng.nextDouble()*6.0));
        for (int i=0;i<n/2;i++) P.add(new Vec2(rng.nextDouble()*8.0, rng.nextDouble()*6.0));
        return P;
    }

    static List<Vec2> scanFromPose(List<Vec2> world, Mat2 R_wb, Vec2 t_wb,
                                  double noiseStd, Random rng){
        Mat2 R_bw = R_wb.T();
        ArrayList<Vec2> scan = new ArrayList<>();
        double maxRange = 10.0;
        double fov = toRadians(270.0);

        for (Vec2 Xw: world){
            Vec2 Xb = R_bw.mul(Xw.sub(t_wb));
            double r = Xb.norm();
            double a = atan2(Xb.y, Xb.x);
            if (r <= maxRange && abs(a) <= fov/2.0){
                double nx = noiseStd * rng.nextGaussian();
                double ny = noiseStd * rng.nextGaussian();
                scan.add(new Vec2(Xb.x + nx, Xb.y + ny));
            }
        }

        int kout = max(5, scan.size()/40);
        for (int i=0;i<kout;i++){
            double ox = -2.0 + 4.0*rng.nextDouble();
            double oy = -2.0 + 4.0*rng.nextDouble();
            scan.add(new Vec2(ox, oy));
        }
        return scan;
    }

    static double angleDeg(Mat2 R){
        return toDegrees(atan2(R.c, R.a));
    }

    public static void main(String[] args){
        Random rng = new Random(7);

        List<Vec2> world = simulateWorld(700, rng);

        Mat2 R0 = Mat2.rot(0.0);
        Vec2 t0 = new Vec2(0.0, 0.0);

        double thetaGT = toRadians(6.0);
        Mat2 R1 = Mat2.rot(thetaGT);
        Vec2 t1 = new Vec2(0.35, 0.10);

        List<Vec2> scanK  = scanFromPose(world, R0, t0, 0.01, new Random(1));
        List<Vec2> scanK1 = scanFromPose(world, R1, t1, 0.01, new Random(2));

        Mat2 Rinit = Mat2.rot(toRadians(3.0));
        Vec2 tinit = new Vec2(0.20, 0.0);

        ICPResult res = icp2D(scanK1, scanK, Rinit, tinit, 60, 1e-7, 0.5, 0.85);

        System.out.println("=== ICP-Based Motion Estimation (2D) ===");
        System.out.printf("Ground truth: theta = %.3f deg, t = [%.3f, %.3f]%n",
                          toDegrees(thetaGT), t1.x, t1.y);
        System.out.printf("Estimated   : theta = %.3f deg, t = [%.3f, %.3f]%n",
                          angleDeg(res.R), res.t.x, res.t.y);
        System.out.printf("Iterations  : %d, final RMSE ≈ %.6f%n", res.iters, res.rmse);
    }
}
