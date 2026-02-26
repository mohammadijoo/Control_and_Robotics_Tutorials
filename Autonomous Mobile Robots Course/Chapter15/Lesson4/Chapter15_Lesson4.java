// Chapter15_Lesson4.java
// Timed-Elastic-Band (TEB) local planning — minimal educational Gauss-Newton (numeric Jacobian).
//
// Requires EJML for linear algebra:
//   https://ejml.org/
// Compile (example):
//   javac -cp ejml-simple-0.43.jar:ejml-core-0.43.jar Chapter15_Lesson4.java
// Run:
//   java  -cp .:ejml-simple-0.43.jar:ejml-core-0.43.jar Chapter15_Lesson4

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class Chapter15_Lesson4 {

    static class CircleObstacle {
        double cx, cy, r;
        CircleObstacle(double cx, double cy, double r) { this.cx = cx; this.cy = cy; this.r = r; }
    }

    static class Pose {
        double x, y, th;
        Pose(double x, double y, double th) { this.x = x; this.y = y; this.th = th; }
    }

    static double wrapToPi(double a) {
        a = (a + Math.PI) % (2.0 * Math.PI);
        if (a < 0) a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    static double hinge(double x) { return (x > 0.0) ? x : 0.0; }

    static class Weights {
        double wSmooth = 10.0;
        double wAlign  = 2.0;
        double wTime   = 1.0;
        double wObst   = 50.0;
        double wV      = 10.0;
        double wW      = 5.0;
        double wA      = 1.0;
        double wAlpha  = 1.0;
    }

    static class Limits {
        double vMax = 0.8;
        double wMax = 1.2;
        double aMax = 0.8;
        double alphaMax = 1.5;
        double dtMin = 0.05;
        double dtMax = 0.6;
        double dMin  = 0.35;
        double robotRadius = 0.20;
    }

    static class ResidualContext {
        Pose start, goal;
        int N;
        List<CircleObstacle> obstacles = new ArrayList<>();
        Weights w = new Weights();
        Limits lim = new Limits();

        void unpack(SimpleMatrix z, double[] xs, double[] ys, double[] th, double[] dts) {
            int nMid = N - 2;
            int nDt  = N - 1;
            xs[0] = start.x; ys[0] = start.y; th[0] = start.th;
            xs[N-1] = goal.x; ys[N-1] = goal.y; th[N-1] = goal.th;

            for (int i = 0; i < nMid; ++i) xs[i+1] = z.get(i, 0);
            for (int i = 0; i < nMid; ++i) ys[i+1] = z.get(nMid + i, 0);
            for (int i = 0; i < nMid; ++i) th[i+1] = z.get(2*nMid + i, 0);
            for (int i = 0; i < nDt;  ++i) dts[i]  = z.get(3*nMid + i, 0);
        }

        SimpleMatrix residual(SimpleMatrix z) {
            double[] xs = new double[N];
            double[] ys = new double[N];
            double[] th = new double[N];
            double[] dts = new double[N-1];
            unpack(z, xs, ys, th, dts);

            ArrayList<Double> r = new ArrayList<>(600);

            // dt objective + bounds
            for (int i = 0; i < N-1; ++i) {
                r.add(Math.sqrt(w.wTime) * dts[i]);
                r.add(Math.sqrt(w.wTime) * hinge(lim.dtMin - dts[i]));
                r.add(Math.sqrt(w.wTime) * hinge(dts[i] - lim.dtMax));
            }

            // smoothness
            for (int i = 1; i < N-1; ++i) {
                double ddx = xs[i+1] - 2.0*xs[i] + xs[i-1];
                double ddy = ys[i+1] - 2.0*ys[i] + ys[i-1];
                r.add(Math.sqrt(w.wSmooth) * ddx);
                r.add(Math.sqrt(w.wSmooth) * ddy);
            }

            // align
            for (int i = 0; i < N-1; ++i) {
                double dx = xs[i+1] - xs[i];
                double dy = ys[i+1] - ys[i];
                double ang = Math.atan2(dy, dx);
                r.add(Math.sqrt(w.wAlign) * Math.sin(wrapToPi(th[i] - ang)));
            }

            // obstacle clearance
            for (int i = 0; i < N; ++i) {
                double px = xs[i], py = ys[i];
                double dmin = 1e9;
                for (CircleObstacle o : obstacles) {
                    double d = Math.hypot(px - o.cx, py - o.cy) - (o.r + lim.robotRadius);
                    dmin = Math.min(dmin, d);
                }
                r.add(Math.sqrt(w.wObst) * hinge(lim.dMin - dmin));
            }

            // v, w bounds + store v,w
            double[] v = new double[N-1];
            double[] ww = new double[N-1];
            for (int i = 0; i < N-1; ++i) {
                double dt = Math.max(dts[i], 1e-6);
                double dx = xs[i+1] - xs[i];
                double dy = ys[i+1] - ys[i];
                double ds = Math.hypot(dx, dy);
                v[i] = ds / dt;
                ww[i] = wrapToPi(th[i+1] - th[i]) / dt;

                r.add(Math.sqrt(w.wV) * hinge(v[i] - lim.vMax));
                r.add(Math.sqrt(w.wW) * hinge(Math.abs(ww[i]) - lim.wMax));
            }

            // accel bounds
            for (int i = 0; i < N-2; ++i) {
                double dtm = 0.5 * (dts[i] + dts[i+1]);
                dtm = Math.max(dtm, 1e-6);
                double a = (v[i+1] - v[i]) / dtm;
                double alpha = (ww[i+1] - ww[i]) / dtm;

                r.add(Math.sqrt(w.wA) * hinge(Math.abs(a) - lim.aMax));
                r.add(Math.sqrt(w.wAlpha) * hinge(Math.abs(alpha) - lim.alphaMax));
            }

            SimpleMatrix out = new SimpleMatrix(r.size(), 1);
            for (int i = 0; i < r.size(); ++i) out.set(i, 0, r.get(i));
            return out;
        }
    }

    static SimpleMatrix finiteDiffJacobian(ResidualContext ctx, SimpleMatrix z, double eps) {
        SimpleMatrix r0 = ctx.residual(z);
        int m = r0.numRows();
        int n = z.numRows();
        SimpleMatrix J = new SimpleMatrix(m, n);

        for (int j = 0; j < n; ++j) {
            SimpleMatrix z1 = z.copy();
            z1.set(j, 0, z1.get(j, 0) + eps);
            SimpleMatrix r1 = ctx.residual(z1);
            SimpleMatrix col = r1.minus(r0).divide(eps);
            for (int i = 0; i < m; ++i) J.set(i, j, col.get(i, 0));
        }
        return J;
    }

    static SimpleMatrix gaussNewton(ResidualContext ctx, SimpleMatrix z0, int maxIter, double lam) {
        SimpleMatrix z = z0.copy();
        for (int it = 0; it < maxIter; ++it) {
            SimpleMatrix r = ctx.residual(z);
            double cost = 0.5 * r.dot(r);

            SimpleMatrix J = finiteDiffJacobian(ctx, z, 1e-6);
            SimpleMatrix A = J.transpose().mult(J).plus(SimpleMatrix.identity(z.numRows()).scale(lam));
            SimpleMatrix b = J.transpose().mult(r).scale(-1.0);

            SimpleMatrix dz = A.solve(b);

            double step = 1.0;
            for (int ls = 0; ls < 10; ++ls) {
                SimpleMatrix zTry = z.plus(dz.scale(step));
                double costTry = 0.5 * ctx.residual(zTry).dot(ctx.residual(zTry));
                if (costTry < cost) { z = zTry; break; }
                step *= 0.5;
            }
            if (dz.normF() * step < 1e-5) break;
        }
        return z;
    }

    public static void main(String[] args) {
        Pose start = new Pose(0.0, 0.0, 0.0);
        Pose goal  = new Pose(4.0, 2.5, 0.0);

        List<CircleObstacle> obstacles = new ArrayList<>();
        obstacles.add(new CircleObstacle(2.0, 1.2, 0.45));
        obstacles.add(new CircleObstacle(2.8, 2.0, 0.35));

        int N = 18;
        double[] xs = new double[N];
        double[] ys = new double[N];
        double[] th = new double[N];
        double[] dts = new double[N-1];

        // initial guess
        for (int i = 0; i < N; ++i) {
            double a = (double)i / (double)(N-1);
            xs[i] = (1.0 - a) * start.x + a * goal.x;
            ys[i] = (1.0 - a) * start.y + a * goal.y;
        }
        th[0] = start.th; th[N-1] = goal.th;
        for (int i = 1; i < N-1; ++i) th[i] = Math.atan2(ys[i+1]-ys[i], xs[i+1]-xs[i]);
        for (int i = 0; i < N-1; ++i) dts[i] = 0.22;

        int nMid = N - 2;
        int nDt  = N - 1;
        SimpleMatrix z0 = new SimpleMatrix(3*nMid + nDt, 1);

        for (int i = 0; i < nMid; ++i) z0.set(i, 0, xs[i+1]);
        for (int i = 0; i < nMid; ++i) z0.set(nMid+i, 0, ys[i+1]);
        for (int i = 0; i < nMid; ++i) z0.set(2*nMid+i, 0, th[i+1]);
        for (int i = 0; i < nDt;  ++i) z0.set(3*nMid+i, 0, dts[i]);

        ResidualContext ctx = new ResidualContext();
        ctx.start = start; ctx.goal = goal; ctx.N = N; ctx.obstacles = obstacles;

        SimpleMatrix zOpt = gaussNewton(ctx, z0, 35, 1e-3);

        double[] xsOpt = new double[N];
        double[] ysOpt = new double[N];
        double[] thOpt = new double[N];
        double[] dtOpt = new double[N-1];
        ctx.unpack(zOpt, xsOpt, ysOpt, thOpt, dtOpt);

        System.out.println("Optimized first 5 points:");
        for (int i = 0; i < Math.min(5, N); ++i) {
            System.out.printf("%02d: (%.3f, %.3f)%n", i, xsOpt[i], ysOpt[i]);
        }
        double totalT = 0.0;
        for (double dt : dtOpt) totalT += dt;
        System.out.println("Total time [s]: " + totalT);
    }
}
