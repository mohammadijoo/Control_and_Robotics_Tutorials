// Chapter15_Lesson1.java
// Pure Pursuit and Geometric Path Tracking (Java 17+)
// Core geometry is library-free; integrate with your robotics stack by wiring pose + path IO.
// (If you use ROS via rosjava, publish Twist commands computed as omega = v * kappa.)

import java.util.ArrayList;
import java.util.List;

final class Vec2 {
    public final double x, y;
    public Vec2(double x, double y) { this.x = x; this.y = y; }
    public Vec2 add(Vec2 b) { return new Vec2(this.x + b.x, this.y + b.y); }
    public Vec2 sub(Vec2 b) { return new Vec2(this.x - b.x, this.y - b.y); }
    public Vec2 mul(double s) { return new Vec2(s * this.x, s * this.y); }
    public double dot(Vec2 b) { return this.x * b.x + this.y * b.y; }
    public double norm() { return Math.hypot(this.x, this.y); }
}

final class Pose2D {
    public final double x, y, theta;
    public Pose2D(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
}

final class PurePursuit {

    public static double wrapAngle(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a >   Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    public static Vec2 worldToBody(Pose2D pose, Vec2 pWorld) {
        // body frame: x forward, y left
        double c = Math.cos(pose.theta);
        double s = Math.sin(pose.theta);
        Vec2 d = new Vec2(pWorld.x - pose.x, pWorld.y - pose.y);
        return new Vec2(c * d.x + s * d.y, -s * d.x + c * d.y);
    }

    public static double[] polylineArcLength(List<Vec2> path, double[] segOut) {
        double[] s = new double[path.size()];
        s[0] = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            Vec2 d = path.get(i + 1).sub(path.get(i));
            segOut[i] = d.norm();
            s[i + 1] = s[i] + segOut[i];
        }
        return s;
    }

    public static Vec2 closestPointOnSegment(Vec2 p, Vec2 a, Vec2 b, double[] tOut) {
        Vec2 ab = b.sub(a);
        double denom = ab.dot(ab);
        if (denom <= 1e-12) { tOut[0] = 0.0; return a; }
        double t = (p.sub(a)).dot(ab) / denom;
        double tc = Math.max(0.0, Math.min(1.0, t));
        tOut[0] = tc;
        return a.add(ab.mul(tc));
    }

    public static class ClosestResult {
        public final Vec2 q; public final int i; public final double t; public final double d;
        public ClosestResult(Vec2 q, int i, double t, double d) { this.q=q; this.i=i; this.t=t; this.d=d; }
    }

    public static ClosestResult closestPointOnPolyline(Vec2 p, List<Vec2> path) {
        double bestD = Double.POSITIVE_INFINITY;
        Vec2 bestQ = path.get(0);
        int bestI = 0;
        double bestT = 0.0;

        for (int i = 0; i < path.size() - 1; i++) {
            double[] tOut = new double[1];
            Vec2 q = closestPointOnSegment(p, path.get(i), path.get(i + 1), tOut);
            double d = p.sub(q).norm();
            if (d < bestD) {
                bestD = d; bestQ = q; bestI = i; bestT = tOut[0];
            }
        }
        return new ClosestResult(bestQ, bestI, bestT, bestD);
    }

    public static Vec2 pointAtArcLength(List<Vec2> path, double[] s, double[] seg, double sQuery) {
        if (sQuery <= 0.0) return path.get(0);
        if (sQuery >= s[s.length - 1]) return path.get(path.size() - 1);

        int j = 0;
        while (j + 1 < s.length && s[j + 1] <= sQuery) j++;
        double ds = sQuery - s[j];
        double t = ds / Math.max(seg[j], 1e-12);
        Vec2 a = path.get(j);
        Vec2 b = path.get(j + 1);
        return a.mul(1.0 - t).add(b.mul(t));
    }

    public static class Output {
        public final double kappa;
        public final Vec2 closest;
        public final Vec2 lookahead;
        public Output(double kappa, Vec2 closest, Vec2 lookahead) { this.kappa=kappa; this.closest=closest; this.lookahead=lookahead; }
    }

    public static Output curvature(Pose2D pose, List<Vec2> path, double Ld) {
        Ld = Math.max(Ld, 1e-3);
        Vec2 p = new Vec2(pose.x, pose.y);

        double[] seg = new double[path.size() - 1];
        double[] s = polylineArcLength(path, seg);

        ClosestResult cr = closestPointOnPolyline(p, path);
        double sClosest = s[cr.i] + cr.t * seg[cr.i];

        Vec2 pLook = pointAtArcLength(path, s, seg, sClosest + Ld);
        Vec2 lookB = worldToBody(pose, pLook);
        double L = Math.hypot(lookB.x, lookB.y);
        double kappa = (L <= 1e-9) ? 0.0 : (2.0 * lookB.y / (L * L));
        return new Output(kappa, cr.q, pLook);
    }

    public static Pose2D bicycleStep(Pose2D pose, double v, double delta, double wheelbase, double dt) {
        double x = pose.x + v * Math.cos(pose.theta) * dt;
        double y = pose.y + v * Math.sin(pose.theta) * dt;
        double th = wrapAngle(pose.theta + (v / wheelbase) * Math.tan(delta) * dt);
        return new Pose2D(x, y, th);
    }
}

public class Chapter15_Lesson1 {
    public static void main(String[] args) {
        // demo path: sinusoid
        List<Vec2> path = new ArrayList<>();
        for (int i = 0; i < 500; i++) {
            double x = 25.0 * i / 499.0;
            double y = 1.8 * Math.sin(0.22 * x);
            path.add(new Vec2(x, y));
        }

        Pose2D pose = new Pose2D(-2.0, -2.0, 0.2);
        double dt = 0.02, wheelbase = 0.33, v = 1.5;
        double LdMin = 0.8, LdMax = 3.5, kV = 0.8;
        double kappaMax = 1.6;
        double deltaMax = Math.toRadians(32.0);

        for (int k = 0; k < (int)(25.0 / dt); k++) {
            double Ld = Math.min(LdMax, Math.max(LdMin, LdMin + kV * Math.abs(v)));
            PurePursuit.Output out = PurePursuit.curvature(pose, path, Ld);

            double kappa = Math.max(-kappaMax, Math.min(kappaMax, out.kappa));
            double delta = Math.atan(wheelbase * kappa);
            delta = Math.max(-deltaMax, Math.min(deltaMax, delta));

            pose = PurePursuit.bicycleStep(pose, v, delta, wheelbase, dt);

            if (k % 50 == 0) {
                System.out.printf("k=%d pose=(%.3f, %.3f, %.3f) kappa=%.4f delta=%.4f%n",
                        k, pose.x, pose.y, pose.theta, kappa, delta);
            }
        }
        System.out.println("Done.");
    }
}
