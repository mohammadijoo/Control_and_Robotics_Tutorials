// Chapter3_Lesson3.java
// Autonomous Mobile Robots - Chapter 3 Lesson 3
// Feasible Path Families for Car-Like Robots (Dubins from scratch; reverse-only variant)
//
// Compile:
//   javac Chapter3_Lesson3.java
// Run:
//   java Chapter3_Lesson3
//
// Output:
//   dubins_path_java.csv

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Chapter3_Lesson3 {

    static final double PI = Math.PI;

    static double mod2pi(double a) {
        a = a % (2.0 * PI);
        if (a < 0.0) a += 2.0 * PI;
        return a;
    }

    static double angdiff(double a, double b) {
        double d = (a - b + PI) % (2.0 * PI);
        if (d < 0.0) d += 2.0 * PI;
        return d - PI;
    }

    static class Pose2 {
        double x, y, theta;
        Pose2(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
    }

    static class DubinsPath {
        String type;      // LSL, RSR, ...
        String segTypes;  // e.g., "LSL"
        double t, p, q;   // normalized: turns are angles [rad]
        double Rmin;

        DubinsPath(String type, String segTypes, double t, double p, double q, double Rmin) {
            this.type = type;
            this.segTypes = segTypes;
            this.t = t; this.p = p; this.q = q;
            this.Rmin = Rmin;
        }

        double length() { return Rmin * (t + p + q); }
    }

    static Pose2 segmentUnit(Pose2 s, char st, double sl) {
        double x = s.x, y = s.y, th = s.theta;
        if (st == 'S') {
            x += sl * Math.cos(th);
            y += sl * Math.sin(th);
            return new Pose2(x, y, th);
        } else if (st == 'L') {
            x += Math.sin(th + sl) - Math.sin(th);
            y += -Math.cos(th + sl) + Math.cos(th);
            th = mod2pi(th + sl);
            return new Pose2(x, y, th);
        } else if (st == 'R') {
            x += -Math.sin(th - sl) + Math.sin(th);
            y += Math.cos(th - sl) - Math.cos(th);
            th = mod2pi(th - sl);
            return new Pose2(x, y, th);
        }
        throw new IllegalArgumentException("Unknown segment type");
    }

    static double endpointError(double alpha, double d, double beta, String seg, double t, double p, double q) {
        Pose2 cur = new Pose2(0.0, 0.0, alpha);
        cur = segmentUnit(cur, seg.charAt(0), t);
        cur = segmentUnit(cur, seg.charAt(1), p);
        cur = segmentUnit(cur, seg.charAt(2), q);
        return Math.hypot(cur.x - d, cur.y) + Math.abs(angdiff(cur.theta, beta));
    }

    interface CandidateFn {
        boolean compute(double alpha, double beta, double d, double[] out);
    }

    // ---- Candidate formulas ----

    static boolean LSL(double alpha, double beta, double d, double[] out) {
        double sa = Math.sin(alpha), sb = Math.sin(beta);
        double ca = Math.cos(alpha), cb = Math.cos(beta);
        double cab = Math.cos(alpha - beta);

        double tmp0 = d + sa - sb;
        double p2 = 2.0 + d*d - 2.0*cab + 2.0*d*(sa - sb);
        if (p2 < 0.0) return false;
        double p = Math.sqrt(p2);
        double tmp1 = Math.atan2(cb - ca, tmp0);
        double t = mod2pi(-alpha + tmp1);
        double q = mod2pi(beta - tmp1);
        out[0] = t; out[1] = p; out[2] = q;
        return true;
    }

    static boolean RSR(double alpha, double beta, double d, double[] out) {
        double sa = Math.sin(alpha), sb = Math.sin(beta);
        double ca = Math.cos(alpha), cb = Math.cos(beta);
        double cab = Math.cos(alpha - beta);

        double tmp0 = d - sa + sb;
        double p2 = 2.0 + d*d - 2.0*cab + 2.0*d*(-sa + sb);
        if (p2 < 0.0) return false;
        double p = Math.sqrt(p2);
        double tmp1 = Math.atan2(ca - cb, tmp0);
        double t = mod2pi(alpha - tmp1);
        double q = mod2pi(-beta + tmp1);
        out[0] = t; out[1] = p; out[2] = q;
        return true;
    }

    static boolean LSR(double alpha, double beta, double d, double[] out) {
        double sa = Math.sin(alpha), sb = Math.sin(beta);
        double ca = Math.cos(alpha), cb = Math.cos(beta);
        double cab = Math.cos(alpha - beta);

        double p2 = -2.0 + d*d + 2.0*cab + 2.0*d*(sa + sb);
        if (p2 < 0.0) return false;
        double p = Math.sqrt(p2);
        double tmp0 = Math.atan2(-ca - cb, d + sa + sb);
        double tmp1 = Math.atan2(-2.0, p);
        double t = mod2pi(-alpha + tmp0 - tmp1);
        double q = mod2pi(-beta + tmp0 - tmp1);
        out[0] = t; out[1] = p; out[2] = q;
        return true;
    }

    static boolean RSL(double alpha, double beta, double d, double[] out) {
        double sa = Math.sin(alpha), sb = Math.sin(beta);
        double ca = Math.cos(alpha), cb = Math.cos(beta);
        double cab = Math.cos(alpha - beta);

        double p2 = -2.0 + d*d + 2.0*cab - 2.0*d*(sa + sb);
        if (p2 < 0.0) return false;
        double p = Math.sqrt(p2);
        double tmp0 = Math.atan2(ca + cb, d - sa - sb);
        double tmp1 = Math.atan2(2.0, p);
        double t = mod2pi(alpha - tmp0 + tmp1);
        double q = mod2pi(beta - tmp0 + tmp1);
        out[0] = t; out[1] = p; out[2] = q;
        return true;
    }

    static boolean RLR(double alpha, double beta, double d, double[] out) {
        double sa = Math.sin(alpha), sb = Math.sin(beta);
        double ca = Math.cos(alpha), cb = Math.cos(beta);
        double cab = Math.cos(alpha - beta);

        double tmp0 = (6.0 - d*d + 2.0*cab + 2.0*d*(sa - sb)) / 8.0;
        if (Math.abs(tmp0) > 1.0) return false;
        double p = mod2pi(2.0*PI - Math.acos(tmp0));
        double tmp1 = Math.atan2(ca - cb, d - sa + sb);
        double t = mod2pi(alpha - tmp1 + p/2.0);
        double q = mod2pi(alpha - beta - t + p);
        out[0] = t; out[1] = p; out[2] = q;
        return true;
    }

    static boolean LRL(double alpha, double beta, double d, double[] out) {
        // symmetry: LRL(alpha,beta,d) = RLR(-alpha,-beta,d)
        return RLR(mod2pi(-alpha), mod2pi(-beta), d, out);
    }

    static DubinsPath dubinsShortest(Pose2 q0, Pose2 q1, double Rmin) {
        if (Rmin <= 0.0) throw new IllegalArgumentException("Rmin must be positive");

        double dx = q1.x - q0.x;
        double dy = q1.y - q0.y;
        double c0 = Math.cos(q0.theta), s0 = Math.sin(q0.theta);

        double x = (c0*dx + s0*dy) / Rmin;
        double y = (-s0*dx + c0*dy) / Rmin;
        double phi = mod2pi(q1.theta - q0.theta);

        double d = Math.hypot(x, y);
        double theta = (d > 0.0) ? Math.atan2(y, x) : 0.0;

        double alpha = mod2pi(-theta);
        double beta  = mod2pi(phi - theta);

        class Cand {
            String type, seg; CandidateFn fn;
            Cand(String type, String seg, CandidateFn fn) { this.type = type; this.seg = seg; this.fn = fn; }
        }

        List<Cand> cands = new ArrayList<>();
        cands.add(new Cand("LSL","LSL", Chapter3_Lesson3::LSL));
        cands.add(new Cand("RSR","RSR", Chapter3_Lesson3::RSR));
        cands.add(new Cand("LSR","LSR", Chapter3_Lesson3::LSR));
        cands.add(new Cand("RSL","RSL", Chapter3_Lesson3::RSL));
        cands.add(new Cand("RLR","RLR", Chapter3_Lesson3::RLR));
        cands.add(new Cand("LRL","LRL", Chapter3_Lesson3::LRL));

        double bestLen = Double.POSITIVE_INFINITY;
        DubinsPath best = null;

        for (Cand c : cands) {
            double[] out = new double[3];
            if (!c.fn.compute(alpha, beta, d, out)) continue;
            double err = endpointError(alpha, d, beta, c.seg, out[0], out[1], out[2]);
            if (err > 1e-6) continue;
            double L = Rmin * (out[0] + out[1] + out[2]);
            if (L < bestLen) {
                bestLen = L;
                best = new DubinsPath(c.type, c.seg, out[0], out[1], out[2], Rmin);
            }
        }

        if (best == null) throw new RuntimeException("No feasible Dubins path found");
        return best;
    }

    static List<Pose2> sample(Pose2 q0, DubinsPath path, double step) {
        List<Pose2> pts = new ArrayList<>();
        pts.add(new Pose2(q0.x, q0.y, mod2pi(q0.theta)));

        Pose2 cur = new Pose2(q0.x, q0.y, mod2pi(q0.theta));
        double[] lens = new double[]{path.t, path.p, path.q};

        for (int i = 0; i < 3; i++) {
            char st = path.segTypes.charAt(i);
            double sl = lens[i];

            if (st == 'S') {
                double L = sl * path.Rmin;
                int n = Math.max(1, (int)Math.ceil(L / step));
                double ds = L / n;
                for (int k = 0; k < n; k++) {
                    cur.x += ds * Math.cos(cur.theta);
                    cur.y += ds * Math.sin(cur.theta);
                    pts.add(new Pose2(cur.x, cur.y, cur.theta));
                }
            } else {
                double a = sl;
                double arc = a * path.Rmin;
                int n = Math.max(1, (int)Math.ceil(arc / step));
                double da = a / n;
                for (int k = 0; k < n; k++) {
                    if (st == 'L') {
                        cur.x += path.Rmin * (Math.sin(cur.theta + da) - Math.sin(cur.theta));
                        cur.y += path.Rmin * (-Math.cos(cur.theta + da) + Math.cos(cur.theta));
                        cur.theta = mod2pi(cur.theta + da);
                    } else {
                        cur.x += path.Rmin * (-Math.sin(cur.theta - da) + Math.sin(cur.theta));
                        cur.y += path.Rmin * (Math.cos(cur.theta - da) - Math.cos(cur.theta));
                        cur.theta = mod2pi(cur.theta - da);
                    }
                    pts.add(new Pose2(cur.x, cur.y, cur.theta));
                }
            }
        }
        return pts;
    }

    static DubinsPath dubinsBackwardOnly(Pose2 q0, Pose2 q1, double Rmin) {
        // drive backward everywhere: equivalent to forward Dubins on headings shifted by pi
        Pose2 q0b = new Pose2(q0.x, q0.y, mod2pi(q0.theta + PI));
        Pose2 q1b = new Pose2(q1.x, q1.y, mod2pi(q1.theta + PI));
        return dubinsShortest(q0b, q1b, Rmin);
    }

    public static void main(String[] args) throws IOException {
        Pose2 q0 = new Pose2(0.0, 0.0, Math.toRadians(10.0));
        Pose2 q1 = new Pose2(8.0, 4.0, Math.toRadians(110.0));
        double Rmin = 2.0;

        DubinsPath fwd = dubinsShortest(q0, q1, Rmin);
        System.out.println("Dubins forward: " + fwd.type + ", length=" + fwd.length());

        DubinsPath back = dubinsBackwardOnly(q0, q1, Rmin);
        System.out.println("Dubins backward-only: " + back.type + ", length=" + back.length());

        List<Pose2> pts = sample(q0, fwd, 0.05);
        try (FileWriter w = new FileWriter("dubins_path_java.csv")) {
            w.write("x,y,theta\n");
            for (Pose2 p : pts) {
                w.write(p.x + "," + p.y + "," + p.theta + "\n");
            }
        }
        System.out.println("Wrote dubins_path_java.csv (" + pts.size() + " samples)");

        // For exact Reeds-Shepp (allowing direction switches), use a library such as OMPL in C++.
    }
}
